# SkiROS2_skill_learning_demo
A demo to learn a contact-rich task with SkiROS2 skills.

This repository contains a set of different things:
* The setup for the whole demo
* The robot setup with a KUKA iiwa
* SkiROS2 skills for a peg insertion
* Scripts for learning and plotting

##  Installation and Setup



## Planning

The peg insertion skill and the goto skill can be used for planning. To do that, select the skill `task_plan` and add the following planning goal:
```pddl
(skiros:at skiros:Product-3 skiros:Container-4)
```