from skiros2_skill.core.skill import SkillDescription, SkillBase, ParallelFf, SerialStar
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element

#################################################################################
# Descriptions
#################################################################################

class ArmMovement(SkillDescription):
    """
    @brief      Any arm movement that brings the end-effector to the target pose
    """

    def createDescription(self):
        # =======Params=========
        self.addParam("Arm", Element("rparts:ArmDevice"), ParamTypes.Required)
        self.addParam("Target", Element("sumo:Object"), ParamTypes.Required)
        self.addParam("Start", Element("sumo:Object"), ParamTypes.Inferred)

        self.addPreCondition(self.getRelationCond("ArmAtStart", "skiros:at", "Arm", "Start", True))
        self.addPreCondition(self.getRelationCond("NotArmAtTarget", "skiros:at", "Arm", "Target", False))
        self.addPostCondition(self.getRelationCond("ArmAtTarget", "skiros:at", "Arm", "Target", True))


class PegInsertion(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("ObservationPose", Element("skiros:ObservationPose"), ParamTypes.Inferred)
        self.addParam("Arm", Element("rparts:ArmDevice"), ParamTypes.Required)
        self.addParam("Container", Element("skiros:Container"), ParamTypes.Required)
        self.addParam("Object", Element("skiros:Product"), ParamTypes.Required)
        self.addParam("Force", 0.0, ParamTypes.Required)
        self.addParam("Radius", 0.0, ParamTypes.Optional)
        self.addParam("PathVelocity", 0.0, ParamTypes.Optional)
        self.addParam("PathDistance", 0.01, ParamTypes.Optional)

        # #=======PreConditions=========
        self.addPreCondition(self.getRelationCond("Holding", "skiros:contain", "Arm", "Object", True))
        self.addPreCondition(self.getRelationCond("NotObjectAtContainer", "skiros:at", "Object", "Container", False))
        self.addPreCondition(self.getRelationCond("ArmAtObservationPose", "skiros:at", "Arm", "ObservationPose", True))
        self.addPreCondition(self.getRelationCond("ContainerHasAObservationPose", "skiros:hasA", "Container", "ObservationPose", True))
        # #=======HoldConditions=========
        self.addHoldCondition(self.getRelationCond("Holding", "skiros:contain", "Arm", "Object", True))
        # #=======PostConditions=========
        self.addPostCondition(self.getRelationCond("ObjectAtContainer", "skiros:at", "Object", "Container", True))
        self.addPostCondition(self.getRelationCond("NotArmInOversationPose", "skiros:at", "Arm", "ObservationPose", False))

class ResetPegInsertion(SkillDescription):
    def createDescription(self):
        self.addParam("ObservationPose", Element("skiros:ObservationPose"), ParamTypes.Inferred)
        self.addParam("Arm", Element("rparts:ArmDevice"), ParamTypes.Required)
        self.addParam("Container", Element("skiros:Container"), ParamTypes.Required)
        self.addParam("Object", Element("skiros:Product"), ParamTypes.Required)

        self.addPreCondition(self.getRelationCond("ContainerHasAObservationPose", "skiros:hasA", "Container", "ObservationPose", True))

#################################################################################
# Implementations
#################################################################################


class go_to_linear(SkillBase):
    def createDescription(self):
        self.setDescription(ArmMovement(), self.__class__.__name__)

    def expand(self, skill):
        skill.setProcessor(SerialStar())
        skill(
        #     self.skill(ParallelFf())(
        #         # self.skill("ArmMovement", "go_to_linear_action"),
        #         self.skill("ChangeStiffness", ""),
        #         self.skill("ApplyForce", "")
        #     ),
            self.skill(ParallelFf())(
                    self.skill("WmSetRelation", "wm_set_relation", remap={'Src': 'Arm', 'Dst': 'Target'},  specify={'Relation': 'skiros:at', 'RelationState': True}),
                    self.skill("WmSetRelation", "wm_set_relation", remap={'Src': 'Arm', 'Dst': 'Start'},  specify={'Relation': 'skiros:at', 'RelationState': False}),
                ),
        )


class peg_insertion(SkillBase):
    def createDescription(self):
        self.setDescription(PegInsertion(), self.__class__.__name__)

    def expand(self, skill):
        skill.setProcessor(SerialStar())
        skill(
                self.skill("WmSetRelation", "wm_set_relation", remap={'Src': 'Arm', 'Dst': 'ObservationPose'},  specify={'Relation': 'skiros:at', 'RelationState': False}),
                self.skill("WmSetRelation", "wm_set_relation", remap={'Src': 'Object', 'Dst': 'Container'},  specify={'Relation': 'skiros:at', 'RelationState': True}),
            )

class reset_peg_insertion(SkillBase):
    def createDescription(self):
        self.setDescription(ResetPegInsertion(), self.__class__.__name__)

    def expand(self, skill):
        skill.setProcessor(SerialStar())
        skill(
            self.skill("WmSetRelation", "wm_set_relation", remap={'Src': 'Arm', 'Dst': 'Container'},  specify={'Relation': 'skiros:at', 'RelationState': True}),
            self.skill("WmSetRelation", "wm_set_relation", remap={'Src': 'Object', 'Dst': 'Container'},  specify={'Relation': 'skiros:at', 'RelationState': False}),
        )