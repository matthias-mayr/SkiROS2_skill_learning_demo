import math
import time
import sys
import signal
import rospy
import textwrap

from skiros2_skill.ros.skill_layer_interface import SkillLayerInterface
from skiros2_common.core.world_element import Element
from skiros2_world_model.ros.world_model_interface import WorldModelInterface
from skiros2_msgs.msg import TreeProgress, SkillProgress
import skiros2_common.core.params as param
import skiros2_common.ros.utils as utils
from cartesian_impedance_controller.msg import ControllerState

from geometry_msgs.msg import Pose

# Translates SkiROS skill states to strings
SkillStateDict = {0: 'unkown', SkillProgress.SUCCESS: 'success', SkillProgress.FAILURE: 'failure', SkillProgress.RUNNING: 'running', SkillProgress.IDLE: 'idle'}


class PegInsertionEpisode(object):
    def __init__(self, params):
        self.params = params
        self.rewards = dict()
        self.rewards["performance"] = 0.0
        self.rewards["force"] = 0.0

        self.state_sub = rospy.Subscriber("/iiwa/CartesianImpedance_trajectory_controller/controller_state", ControllerState, self.state_callback)
        self.force = None
        self.ee_pose = None
        self.ref_pose = Pose()
        self.ref_pose.position.x = 0.5
        self.ref_pose.position.y = 0.0
        self.ref_pose.position.z = 0.05


    def state_callback(self, msg):
        self.force = msg.commanded_wrench.force.z
        self.ee_pose = msg.current_pose

    def calculate_rewards(self):
        if self.force is not None:
            self.rewards["force"] -= abs(self.force)
        if self.ee_pose is not None:
            def calculate_distance(pose1, pose2):
                dx = pose1.position.x - pose2.position.x
                dy = pose1.position.y - pose2.position.y
                dz = pose1.position.z - pose2.position.z
                return math.sqrt(dx**2 + dy**2 + dz**2)
            distance = -calculate_distance(self.ee_pose, self.ref_pose)
            self.rewards["performance"] += 10*(0.3 - min(0.3, distance))

    def final_bt_signal(self, signal):
        if signal == SkillProgress.SUCCESS:
            self.rewards["performance"] += 100
        elif signal == SkillProgress.FAILURE:
            self.rewards["performance"] -= 100

    def get_rewards(self):
        return self.rewards
    
    def get_negative_rewards(self):
        cost = {}
        for key, value in self.rewards.items():
            cost[key] = -value
        return cost


class SkirosRlClient(object):
    def __init__(self):
        self.skill_name = rospy.get_param("~skill_name", "peg_insertion")
        self.reset_skill_name = rospy.get_param("~reset_skill_name", "reset_peg_insertion")
        self.verbose = rospy.get_param("~verbose", False)

        self.task_id = 0
        self.reset_id = None
        self.bt_responses = dict()
        self.sli = SkillLayerInterface("rl_skill_layer")
        self.sli.set_monitor_cb(self.process_tree_progress)
        while "/iiwa_robot" not in self.sli.agents:
            time.sleep(0.1)
        self.agent = self.sli.agents["/iiwa_robot"]
        rospy.loginfo("Robot appeared.")

        self.wmi = WorldModelInterface()

        self.arm = self.get_wm_element("rparts:ArmDevice", "iiwa Arm")
        self.container = self.get_wm_element("skiros:Container", "Box with hole")
        self.object = self.get_wm_element("skiros:Product", "Peg")


    def get_wm_element(self, element_type, element_name):
        element = self.wmi.resolve_element(Element(element_type, element_name))
        while element is None:
            rospy.logerr("Could not resolve world model element for '%s'.", element_name)
            time.sleep(0.1)
            element = self.wmi.resolve_element(Element(element_type, element_name))
        return element

    def set_peg_insertion_parameters(self, skill, params = None):
        
        skill.ph["Arm"].value = self.arm
        skill.ph["Container"].value = self.container
        skill.ph["Object"].value = self.object

        if params is not None:
            for key, value in params.items():
                if type(value) is np.float32:
                    value = float(value)
                skill.ph[key].value = value


    def run_episode(self, params = None, max_time = 15.0):
        # Make sure the reset skill has finished
        if self.reset_id is not None and self.reset_id in self.bt_responses:
            while self.bt_responses[self.reset_id] != SkillProgress.SUCCESS:
                rospy.sleep(0.1)
                rospy.loginfo_throttle(1, "Waiting for reset to finish.")
        
        print("Starting episode with parameters: ", params)
        start_time = rospy.Time.now()
        
        skill = self.agent.get_skill(self.skill_name)
        self.set_peg_insertion_parameters(skill, params)
            
        episode = PegInsertionEpisode(params)
        self.task_id = self.agent.execute(skill_list=[skill])

        # Wait for the task to finish with max_time timeout
        while (rospy.Time.now() - start_time).to_sec() < max_time:
            episode.calculate_rewards()
            rospy.sleep(0.1)

        if self.task_id in self.bt_responses:
            rospy.loginfo("Received bt response: %s", SkillStateDict[self.bt_responses[self.task_id]])
            if self.bt_responses[self.task_id] == SkillProgress.RUNNING:
                rospy.loginfo("Preempting task.")
                self.agent.preempt_one(self.task_id)
            episode.final_bt_signal(self.bt_responses[self.task_id])
        print("Rewards: ", episode.get_rewards())
            
        reset_skill = self.agent.get_skill(self.reset_skill_name)
        self.set_peg_insertion_parameters(reset_skill)
        self.reset_id = self.agent.execute(skill_list=[reset_skill])
        return episode.get_negative_rewards()

    # Processes the feedback from SkiROS.
    def process_tree_progress(self, progress):
        tree_progress = progress.progress
        if self.verbose:
            self.print_treeprogress(tree_progress)
        for msg in tree_progress:
            # The root tells us about the state of the whole tree
            if msg.type == ":Root":
                self.bt_responses[msg.id] = msg.state


    def print_treeprogress(self, tree_progress):
        # Debugging information
        for skillprogress in tree_progress:
            int_1 = "    "
            int_2 = 2 * int_1
            print_params = True

            rospy.loginfo("==> Feedback robot: %s task_id: %i id: %i",
                          skillprogress.robot, skillprogress.task_id, skillprogress.id)
            rospy.loginfo("%sType: %s Label: %s", int_1, skillprogress.type, skillprogress.label)

            rospy.loginfo("%sState: %s Progress Code: %i  Progress message: %s", int_1,
                          SkillStateDict[skillprogress.state], skillprogress.progress_code,
                          skillprogress.progress_message)

            params_des = utils.deserializeParamMap(skillprogress.params)
            wrapper = textwrap.TextWrapper()
            wrapper.initial_indent = int_2
            wrapper.subsequent_indent = int_2
            wrapper.width = 100
            if print_params:
                # print wrapper.fill(text=str(skillprogress.params))
                for key, value in params_des.items():
                    rospy.loginfo("%s%s : %s", int_2, key, value.printState())


def main():
    # Allows to exit upon ctrl-c
    def signal_handler(signal, frame):
        print("\nProgram exiting gracefully")
        rl_client.shutdown()
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node("rl_client")
    rl_client = SkirosRlClient()
    rl_client.run_episode()


if __name__ == "__main__":
    main()