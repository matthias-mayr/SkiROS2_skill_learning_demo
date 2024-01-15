# Use ROS Noetic base image
FROM ros:noetic-ros-base
RUN apt-get update && apt-get install -y python3-catkin-tools
RUN apt-get update && apt-get install -y git python3-pip

# Set the working directory in the container
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; catkin init"
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; catkin build"

# Copy the install script into the container
COPY scripts/installation.sh .

# Give execution rights to the install script
RUN chmod +x installation.sh

# Execute the install script
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; ./installation.sh /catkin_ws"
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; cd /catkin_ws && catkin build"

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]