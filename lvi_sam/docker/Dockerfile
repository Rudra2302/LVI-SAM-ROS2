FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

# Install ROS2 dependencies and tools
RUN apt update && apt install -y \
    build-essential \
    python3-colcon-common-extensions \
    python3-pip \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    libpcl-dev \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# Setup workspace
RUN mkdir -p /home/ros2_ws/src
WORKDIR /home/ros2_ws

COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
