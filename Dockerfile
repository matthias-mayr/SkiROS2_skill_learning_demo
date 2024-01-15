# Use ROS Noetic base image
FROM ros:noetic-ros-base

# Set the working directory in the container
RUN mkdir /catkin_ws/
WORKDIR /catkin_ws

# Copy the install script into the container
COPY scripts/installation.sh .

# Give execution rights to the install script
RUN chmod +x installation.sh

# Execute the install script
RUN ./installion.sh /catkin_ws

ENTRYPOINT ["bash"]