# Use ROS Noetic base image
FROM ros:noetic-ros-base
RUN apt-get update && apt-get install -y git python3-catkin-tools
# here temporarily - done by installation.sh eventually:
RUN apt-get update && apt-get install -y git tmux python3-pip python3-catkin-tools ros-noetic-rosmon

# Set the working directory in the container
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; catkin init"
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; catkin build"
RUN git clone https://github.com/matthias-mayr/SkiROS2_skill_learning_demo.git src/SkiROS2_skill_learning_demo

# Execute the install script
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; /catkin_ws/src/SkiROS2_skill_learning_demo/scripts/installation.sh /catkin_ws"
# Do not build the driver - most people do not have the FRI library
RUN touch /catkin_ws/src/iiwa_ros/iiwa_driver/CATKIN_IGNORE
# Build the catkin workspace
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; cd /catkin_ws && catkin build"

ENTRYPOINT ["/bin/bash", "--init-file", "/catkin_ws/devel/setup.bash"]