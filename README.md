# SkiROS2_skill_learning_demo
A demo to learn a contact-rich task with SkiROS2 skills.

This repository contains a set of different things:
* The setup for the whole demo
* The robot setup with a KUKA iiwa
* SkiROS2 skills for a peg insertion
* Scripts for learning and plotting

##  Installation and Setup
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init && catkin build
git clone https://github.com/matthias-mayr/SkiROS2_skill_learning_demo.git
SkiROS2_skill_learning_demo/scripts/installation.sh ~/catkin_ws
catkin build
source devel/setup.bash
```

## Startup
Launch the robot setup with
```
mon launch skiros2_skill_learning_demo robot.launch
```

Launch SkiROS2 with 
```
mon launch skiros2_skill_learning_demo skiros.launch
```

## Planning

The peg insertion skill and the goto skill can be used for planning. To do that, select the skill `task_plan` and add the following planning goal:
```pddl
(skiros:at skiros:Product-3 skiros:Container-4)
```

# Learning