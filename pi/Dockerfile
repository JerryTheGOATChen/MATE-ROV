FROM ros:jazzy-ros-base-noble
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake python3-colcon-common-extensions \
    ros-jazzy-rclcpp ros-jazzy-sensor-msgs ros-jazzy-std-msgs \
    ros-jazzy-launch ros-jazzy-launch-ros ros-jazzy-ament-index-python \
    ros-jazzy-ros2param ros-jazzy-ros2topic \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /workspace
CMD ["bash"]
