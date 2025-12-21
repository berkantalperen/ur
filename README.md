# Base System Bringup (UR5e + URSim + ROS 2 + MoveIt)

This setup verifies that **URSim, ROS 2 driver, and MoveIt execution** are working correctly **before adding any custom workspaces or nodes**.

## Environment
- Robot: UR5e  
- Simulator: URSim (Docker)  
- Control: ur_robot_driver (ROS 2)  
- Planning: MoveIt 2  
- External Control URCap required  

---

## 0. Start URSim (Docker)
```bash
docker start ursim_ur5e
```

Get container IP:
```bash
docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' ursim_ur5e
```

Open Polyscope:
```
http://localhost:6080/vnc.html
```

---

## 1. Prepare Polyscope (URSim UI)
1. Ensure **External Control URCap** is installed  
   Settings → System → URCaps  
2. Create or load a program with **External Control** node  
3. **Do NOT press Play yet**

---

## 2. Start ROS 2 UR Driver
```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  robot_ip:=172.17.0.2 \
  launch_rviz:=false \
  initial_joint_controller:=scaled_joint_trajectory_controller
```

Wait until you see:
```
System successfully started!
```

---

## 3. Start External Control (Polyscope)
Now go back to Polyscope and press **Play** on the External Control program.

Verify from ROS:
```bash
ros2 topic echo /io_and_status_controller/robot_program_running --once
```

Expected:
```
data: true
```

Check controller state:
```bash
ros2 control list_controllers | grep trajectory
```

Expected:
```
scaled_joint_trajectory_controller ... active
```

---

## 4. Start MoveIt (with RViz)
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur5e \
  launch_rviz:=true
```

You can now:
- Plan
- Execute
- See motion in URSim

---

## Status
- URSim running  
- ROS 2 driver connected  
- External Control active  
- MoveIt planning & execution working  
- No custom workspace yet  

This state is the **project baseline**.
