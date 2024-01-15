# Use ROS Noetic base image
FROM ros:noetic-ros-base
RUN apt-get update && apt-get install -y python3-catkin-tools

# Set the working directory in the container
RUN mkdir /catkin_ws/
WORKDIR /catkin_ws
RUN catkin init
RUN catkin build

# Copy the install script into the container
COPY scripts/installation.sh .

# Give execution rights to the install script
RUN chmod +x installation.sh

# Execute the install script
RUN ./installation.sh /catkin_ws

ENTRYPOINT ["bash"]