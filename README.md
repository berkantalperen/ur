 # UR5e Teleoperation (ROS 2 Jazzy)

This repository contains ROS 2 workspaces for working with a **Universal Robots UR5e**
using **URSim**, **ur_robot_driver**, **MoveIt 2**, and custom teleoperation/control nodes.

## Structure

```text
ur/
├── teleoperation/        # Project workspace (custom nodes)
│   └── src/
│       ├── ur_cpp_control
│       └── ur_scripts
├── ur5e_config/          # (Optional) custom MoveIt config
│   └── src/
│       └── ur_moveit_config
