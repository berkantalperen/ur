# File: /ur/Dockerfile
FROM osrf/ros:jazzy-desktop

# 1. Install System Dependencies
# - python3-pyqt5: Required for ur_dashboard_ui
# - ros-jazzy-ur-robot-driver & moveit: Required for ur_cpp_control
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-pyqt5 \
    ros-jazzy-ur-robot-driver \
    ros-jazzy-ur-moveit-config \
    ros-jazzy-moveit \
    ros-jazzy-moveit-ros-planning-interface \
    git \
    && rm -rf /var/lib/apt/lists/*

# 2. Setup the Workspace Directory
# We do NOT copy code here; we mount it later.
WORKDIR /root/ur_ws

# 3. Source ROS automatically
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

CMD ["bash"]