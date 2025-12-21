# ur_dashboard_ws

A minimal ROS 2 workspace that provides a Qt-based dashboard UI for the Universal Robots driver. The UI discovers available dashboard services at runtime, lists them, and lets you call them from a simple panel.

## Prerequisites

- ROS 2 (same distro as the driver)
- `colcon` tooling
- Qt Python bindings (`PyQt5` or `PySide6`)

Example (Ubuntu + ROS 2 apt packages):

```bash
sudo apt install python3-pyqt5
```

## Build

```bash
cd /workspace/ur/ur_dashboard_ws
colcon build --symlink-install
source install/setup.bash
```

## Run the UR driver (required)

This mirrors the **Step 1** instructions from `/workspace/ur/ur_cpp_control_ws/ur_cpp_control_ws_readme.md`:

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  robot_ip:=<ROBOT_IP> \
  launch_rviz:=true
```

Verify controllers are active:

```bash
ros2 control list_controllers
```

## Run the dashboard UI

```bash
source /workspace/ur/ur_dashboard_ws/install/setup.bash
ros2 launch ur_dashboard_ui ur_dashboard_ui.launch.py
```

The UI will:

- Discover dashboard services (`ros2 service list | rg dashboard` behavior)
- List each service name and type on the left
- Show action details on the right
- Let you call supported service types (e.g., `std_srvs/srv/Trigger`)

## Verify calls

1. Start the UR driver (above)
2. Launch the UI
3. Select a dashboard service (e.g., `/dashboard/play`)
4. Click **Call** and observe the response in the output panel

If a service type is not supported yet, the UI will show it but disable the **Call** button.
