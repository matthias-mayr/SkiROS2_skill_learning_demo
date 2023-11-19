from skiros2_skill.core.skill import SkillDescription, SkillBase, ParallelFs, Serial
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element

#################################################################################
# Descriptions
#################################################################################

class MySkill(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("WorldModelObject", Element("skiros:TransformationPose"), ParamTypes.Required)

#################################################################################
# Implementations
#################################################################################

class my_skill(SkillBase):
    """
    Tree is:
    ----->:Skill (->)
    ------->:MyPrimitive

    """
    def createDescription(self):
        self.setDescription(MySkill(), self.__class__.__name__)

    def expand(self, skill):
        skill.setProcessor(Serial())
        skill(
            self.skill("MyPrimitive", "my_primitive")
        )
