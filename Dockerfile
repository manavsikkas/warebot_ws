# Use the core ROS image (Guaranteed to have ARM64 support)
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Install the Desktop tools and Gazebo manually
RUN apt-get update && apt-get install -y \
    ros-jazzy-desktop \
    ros-jazzy-ros-gz \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-gz-ros2-control \
    nano \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /warebot_ws

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
