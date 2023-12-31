from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.conditions import ConditionHasProperty
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_std_skills.action_client_primitive import PrimitiveActionClient

from copy import deepcopy
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
import geometry_msgs.msg
from cartesian_trajectory_generator.msg import TrajectoryAction, TrajectoryGoal
import cartesian_trajectory_generator.srv

#################################################################################
# Descriptions
#################################################################################


class ArmMovementAction(SkillDescription):
    """
    @brief      Any arm movement that brings the end-effector to the target pose
    """

    def createDescription(self):
        # =======Params=========
        self.addParam("Arm", Element("rparts:ArmDevice"), ParamTypes.Required)
        self.addParam("Target", Element("sumo:Object"), ParamTypes.Required)


class ChangeStiffness(SkillDescription):
    """
    @brief Change end effector stiffness.
    """

    def createDescription(self):
        self.addParam("Arm", Element("rparts:ArmDevice"), ParamTypes.Required)
        self.addParam("TransX", 400.0, ParamTypes.Optional)
        self.addParam("TransY", 400.0, ParamTypes.Optional)
        self.addParam("TransZ", 400.0, ParamTypes.Optional)
        self.addParam("RotX", 40.0, ParamTypes.Optional)
        self.addParam("RotY", 40.0, ParamTypes.Optional)
        self.addParam("RotZ", 40.0, ParamTypes.Optional)


class ApplyForce(SkillDescription):
    """
    @brief Apply force with the end effector.
    """

    def createDescription(self):
        self.addParam("Arm", Element("rparts:ArmDevice"), ParamTypes.Required)
        self.addParam("TransX", 0.0, ParamTypes.Optional)
        self.addParam("TransY", 0.0, ParamTypes.Optional)
        self.addParam("TransZ", 0.0, ParamTypes.Optional)
        self.addParam("RotX", 0.0, ParamTypes.Optional)
        self.addParam("RotY", 0.0, ParamTypes.Optional)
        self.addParam("RotZ", 0.0, ParamTypes.Optional)


class OverlayMotion(SkillDescription):
    """
    @brief Apply Overlay motion with the end effector.
    """

    def createDescription(self):
        self.addParam("Arm", Element("rparts:ArmDevice"), ParamTypes.Required)
        self.addParam("Motion", str, ParamTypes.Required)
        self.addParam("Radius", float, ParamTypes.Optional)
        self.addParam("PathDistance", float, ParamTypes.Optional)
        self.addParam("PathVelocity", float, ParamTypes.Optional)
        self.addParam("AllowDecrease", bool, ParamTypes.Optional)
        self.addParam("Dir", float, ParamTypes.Optional)


#################################################################################
# Implementations
#################################################################################

class go_to_linear_action(PrimitiveActionClient):
    """
    @brief Move arm directly to target (aligns x y z)
    """

    def createDescription(self):
        self.setDescription(ArmMovementAction(), self.__class__.__name__)

    def buildClient(self):
        if not self.params["Arm"].value.hasProperty("skiros:CartesianGoalAction"):
            rospy.logerr("Arm does not have required property 'skiros:CartesianGoalAction'")
            return None
        action_topic = self.params["Arm"].value.getProperty("skiros:CartesianGoalAction").value
        return actionlib.SimpleActionClient(action_topic,
                                            TrajectoryAction)

    def transform_to_frame(self, element, target_frame):
        if not element.hasProperty("skiros:FrameId", not_none=True):
            raise Exception("Missing pose info for target.")
        reasoner = element._getReasoner("AauSpatialReasoner")
        reasoner.get_transform(element.getProperty("skiros:FrameId").value, target_frame)
        reasoner.transform(element, target_frame)
        return element

    def buildGoal(self):
        target = self.params["Target"].value
        goal = TrajectoryGoal()
        if not target.hasProperty("skiros:FrameId", not_none=True):
            raise Exception("Missing frame_id of goal")
        parent_frame = target.getProperty("skiros:BaseFrameId").value
        target_msg = self.transform_to_frame(
            deepcopy(target), parent_frame).getData(":PoseStampedMsg")
        goal.header.frame_id = target.getProperty("skiros:BaseFrameId").value
        goal.header.stamp = rospy.Time.now()
        goal.goal = target_msg
        return goal

    def onFeedback(self, msg):
        return self.step("Progress: {}%. Trans-error: {:.3f} Rot-error: {:.2f}".format(
            round(100 * msg.time_percentage), msg.trans_goal_error, msg.rot_goal_error))

    def onDone(self, status, msg):
        if status == GoalStatus.ABORTED:
            return self.fail("Failed aborted", -2)
        elif status == GoalStatus.SUCCEEDED:
            return self.success("Succeeded")
        elif status == GoalStatus.REJECTED:
            rospy.loginfo(status)
            rospy.loginfo(msg)
            return self.fail("Goal was rejected by action server.", -2)
        else:
            rospy.loginfo(status)
            rospy.loginfo(msg)
            return self.fail("Unknown return code.", -100)

class change_stiffness(PrimitiveBase):
    """
    @brief Commands a robot controller to apply a stiffness

    Publishes a geometry_msgs:WrenchStamped message to the specified topic.

    @return - Success:  After n seconds
            - Fail:     Never
            - Running:  Until n seconds
    """

    def createDescription(self):
        self.setDescription(ChangeStiffness(), self.__class__.__name__)

    def modifyDescription(self, skill):
        skill.addPreCondition(ConditionHasProperty("ArmHasCartesianStiffnessTopic", "skiros:CartesianStiffnessTopic", "Arm", True))

    def onInit(self):
        self.topic = ""
        self.prop = "skiros:CartesianStiffnessTopic"

    def onStart(self):
        if not self.params["Arm"].value.hasProperty(self.prop):
            rospy.logerr("Arm does not have required property %s", self.prop)
            return False
        topic = self.params["Arm"].value.getProperty(self.prop).value
        if topic != self.topic:
            self.topic = topic
            self.pub = rospy.Publisher(
                self.topic, geometry_msgs.msg.WrenchStamped, queue_size=10)
            rospy.sleep(0.2) # Wait for publisher to be ready
        msg = geometry_msgs.msg.WrenchStamped()
        msg.header.stamp = rospy.Time.now()
        msg.wrench.force.x = self.params["TransX"].value
        msg.wrench.force.y = self.params["TransY"].value
        msg.wrench.force.z = self.params["TransZ"].value
        msg.wrench.torque.x = self.params["RotX"].value
        msg.wrench.torque.y = self.params["RotY"].value
        msg.wrench.torque.z = self.params["RotZ"].value
        self.pub.publish(msg)

        self.start_time = rospy.Time.now()
        self.max_time = rospy.Duration(0.0)
        return True

    def execute(self):
        if (rospy.Time.now() - self.start_time) > self.max_time:
            return self.success("Assuming stiffness change has worked")
        return self.step("Changing stiffness.")


class apply_force(PrimitiveBase):
    """
    @brief Applies a force with the end effector. Persistent skill.

    Publishes the force values to a specified topic as geometry_msgs:WrenchStamped.

    @return - Success:  Never
            - Fail:     Never
            - Running:  Always
    """

    def createDescription(self):
        self.setDescription(ApplyForce(), self.__class__.__name__)

    def modifyDescription(self, skill):
        skill.addPreCondition(ConditionHasProperty("ArmHasCartesianWrenchTopic", "skiros:CartesianWrenchTopic", "Arm", True))

    def onInit(self):
        self.topic = ""
        self.prop = "skiros:CartesianWrenchTopic"

    def onPreempt(self):
        self.msg = geometry_msgs.msg.WrenchStamped()
        self.msg.header.stamp = rospy.Time.now()
        self.pub.publish(self.msg)
        return self.success("Set force to 0")

    def onStart(self):
        if not self.params["Arm"].value.hasProperty(self.prop):
            rospy.logerr("Arm does not have required property %s", self.prop)
            return False
        topic = self.params["Arm"].value.getProperty(self.prop).value
        if topic != self.topic:
            self.topic = topic
            self.pub = rospy.Publisher(
                self.topic, geometry_msgs.msg.WrenchStamped, queue_size=10)
            rospy.sleep(0.2) # Wait for publisher to be ready
        self.msg = geometry_msgs.msg.WrenchStamped()
        self.msg.header.stamp = rospy.Time.now()
        self.msg.wrench.force.x = self.params["TransX"].value
        self.msg.wrench.force.y = self.params["TransY"].value
        self.msg.wrench.force.z = self.params["TransZ"].value
        self.msg.wrench.torque.x = self.params["RotX"].value
        self.msg.wrench.torque.y = self.params["RotY"].value
        self.msg.wrench.torque.z = self.params["RotZ"].value
        self.pub.publish(self.msg)
        return True

    def execute(self):
        return self.step("Changing to wrench {}.".format(self.msg.wrench))


class overlay_motion(PrimitiveBase):
    """
    @brief Applies an overlay motion with the trajectory generator. Persistent skill.

    This applies an overlay motion while the skill is running. It uses the Cartesian trajectory generator.
    The overlay motion is automatically cancelled when the skill is preempted.

    @return - Success:  Never
            - Fail:     Never. Wrong motions abort the start
            - Running:  Always
    """

    def createDescription(self):
        self.setDescription(OverlayMotion(), self.__class__.__name__)

    def modifyDescription(self, skill):
        skill.addPreCondition(ConditionHasProperty("ArmHasOverlayMotionService", "skiros:OverlayMotionService", "Arm", True))

    def onInit(self):
        self.topic = ""
        self.prop = "skiros:OverlayMotionService"
        self.serv_obj = None

    def onPreempt(self):
        req = cartesian_trajectory_generator.srv.OverlayMotionRequest()
        req.motion = req.NONE
        self.serv_obj(req)

    def onStart(self):
        if not self.params["Arm"].value.hasProperty(self.prop):
            rospy.logerr("Arm does not have required property %s", self.prop)
            return False
        topic = self.params["Arm"].value.getProperty(self.prop).value
        if topic != self.topic or self.serv_obj is None:
            self.topic = topic
            rospy.wait_for_service(self.topic)
            self.serv_obj = rospy.ServiceProxy(
                self.topic, cartesian_trajectory_generator.srv.OverlayMotion)

        req = cartesian_trajectory_generator.srv.OverlayMotionRequest()
        if self.params['Motion'].value == "archimedes":
            req.motion = req.ARCHIMEDES
        elif self.params['Motion'].value == "":
            req.motion = req.NONE
        else:
            rospy.logerr("Unknown overlay motion %s", self.params['Motion'].value)

        req.radius = self.params['Radius'].value
        req.path_distance = self.params['PathDistance'].value
        req.path_velocity = self.params['PathVelocity'].value
        req.allow_decrease = self.params['AllowDecrease'].value
        dir = self.params["Dir"].values
        req.dir.x = dir[0]
        req.dir.y = dir[1]
        req.dir.z = dir[2]
        self.serv_obj(req)
        self.start_time = rospy.Time.now()
        self.max_time = rospy.Duration(2.0)
        return True

    def execute(self):
        return self.step("Applying Overlay motion.")