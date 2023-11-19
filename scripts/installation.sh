#!/bin/bash

# check if ROS noetic is installed
if [ -z "$ROS_DISTRO" ]; then
    echo "ROS noetic is not installed. Please install it first."
    exit 1
fi

# check if the workspace is given as a first argument
if [ -z "$1" ]; then
    echo "Please provide the workspace as a first argument."
    exit 1
fi

# check if the workspace exists
if [ ! -d "$1" ]; then
    echo "The workspace does not exist."
    exit 1
fi

# check if the workspace is a catkin workspace
if [ ! -f "$1"/devel/setup.bash ]; then
    echo "The workspace is not a catkin workspace."
    exit 1
fi

ws=$(readlink -f "$1")

# clone packages
cd "$ws"/src
git clone https://github.com/epfl-lasa/iiwa_ros
git clone https://github.com/RVMI/skiros2
git clone https://github.com/RVMI/skiros2_std_lib

git clone https://github.com/matthias-mayr/Cartesian-Impedance-Controller
git clone https://github.com/matthias-mayr/cartesian_trajectory_generator

### install dependencies
# SkiROS2
cd skiros2
pip install -r requirements.txt --user

# General dependencies
cd "$ws"
rosdep install --from-paths src --ignore-src -r -y

# Cartesian impedance controller dependencies
src/Cartesian-Impedance-Controller/scripts/install_dependencies.sh
cd depends

# Additional iiwa_ros dependencies
git clone https://github.com/mosra/corrade.git
cd corrade
git checkout 0d149ee9f26a6e35c30b1b44f281b272397842f5
mkdir build && cd build
cmake ..
make -j
sudo make install
cd ../..

git clone https://github.com/epfl-lasa/robot_controllers.git
cd robot_controllers
mkdir build && cd build
cmake ..
make -j
sudo make install