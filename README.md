# Portable UR Development Environment (Docker)

This repository includes a portable **Docker-based development environment** for Universal Robots (UR5e), ROS 2 Jazzy, and MoveIt. It encapsulates all system dependencies (Qt, MoveIt, Drivers) in a container while mounting your source code so you can edit files natively on your host machine.

## Features

* **Portable:** Runs on Ubuntu (Native) and Windows 10/11 (Docker Desktop).
* **Consistent:** Everyone uses the exact same version of ROS 2 Jazzy, MoveIt, and PyQt5.
* **Development Ready:** Source code is mounted from your host. Edit in VS Code on Windows/Linux; build inside the container instantly.
* **Hardware Capable:** configured for direct Ethernet communication with real UR robots.

---

## 1. One-Time Setup

Create the following three files in the root of your repository (next to `ur_cpp_control_ws` and `ur_dashboard_ws`).

### A. `Dockerfile` (The Toolset)
Installs ROS 2, MoveIt, and your specific package dependencies.

```dockerfile
# File: ./Dockerfile
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