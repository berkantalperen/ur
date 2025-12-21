# ur_cpp_control_ws

A minimal **Plan → Execute** demonstration for a **UR5e** using **ROS 2** + **Universal Robots ROS 2 Driver**, designed to keep the flow *explicit*:

1) start the UR driver  
2) start the **execute service** (executor server)  
3) run the **planner** to generate a plan  
4) trigger **execution** of the latest plan

> ⚠️ **Demo scope**
> - This is only a **demonstration** of a plan–execute architecture.
> - The goal is currently **hard‑coded** in the planner node.
> - Later: goal becomes a **parameter**, and a **UI** can set it (and visualize it).

---

## What you get

- A **planner node** that computes a trajectory (currently to a hard‑coded target)
- An **executor service node** that accepts a trajectory and sends it to the controller
- A **trigger mechanism** (service/topic/action depending on your implementation) to execute the latest plan
- Optional **URSim** usage for safe testing

---

## Optional: URSim vs real robot

You can run the exact same pipeline against:

- **URSim** (recommended while iterating)
- **Real UR5e** (when you’re ready)

As long as the **UR driver** is connected and the required controllers are active, the plan/execute architecture behaves the same.

---

## How to run (explanation + commands together)

This section explains **what each step does** and shows **the exact command** to run it. Follow in order.

---

### 1) Start the UR driver (URSim or real robot)

**What this does**  
Bridges ROS 2 with the robot (or URSim), exposes controllers, joint states, and dashboard services. Nothing else works without this.

**Command**

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  robot_ip:=<ROBOT_IP> \
  launch_rviz:=true
```

**Check**

```bash
ros2 control list_controllers
```

You must see:
- `joint_state_broadcaster` → active
- `joint_trajectory_controller` → active

---

### 2) Start the execute service (executor node)

**What this does**  
Starts a server that knows **how to send trajectories to the controller**. It does nothing until you explicitly tell it to execute a plan.

**Command**

```bash
ros2 run ur_cpp_control execute_node
```

At this point:
- Robot does **not** move
- Executor waits idle for a plan + trigger

---

### 3) Run the planner node

**What this does**  
Computes a trajectory to a **hard‑coded target** but does not move the robot. This separation is intentional.

**Command**

```bash
ros2 run ur_cpp_control plan_node
```

**What you see**
- RViz shows a **purple animated trajectory preview**
- Robot still does **not** move

This allows you to validate the motion before execution.

---

### 4) Trigger execution of the last plan

**What this does**  
Explicitly tells the executor to send the **latest planned trajectory** to the controller.

**Command**

```bash
ros2 service call /execute_plan std_srvs/srv/Trigger {}
```

**Result**
- Trajectory is streamed to `joint_trajectory_controller`
- Robot moves

---

## RViz visualization notes


### Purple “animation” when a plan is sent
When using MoveIt planning / trajectory display, sending a plan to the visualization pipeline shows:
- a **purple trajectory** / animated preview in RViz

This is great for sanity checks:
- you can confirm the path looks correct before moving the real robot

### Simultaneous usage
You can typically:
- keep RViz open to monitor state and preview plans
- run the plan/execute nodes in parallel

---

## Future: show planner target in RViz

Later improvement ideas:

- Publish the target as a topic (e.g., a `PoseStamped` marker frame) so RViz can display:
  - a target axis/arrow marker
  - a ghost end‑effector goal pose
- Publish the planned end state as a marker or TF frame
- Optionally visualize multiple candidate goals / queued goals

This makes the demo become a UI-driven system:
- UI sets goal → planner publishes target marker → RViz shows it → planner computes plan → preview in RViz → execute

---

## Common pitfalls

- Forgot to **source** the workspace in a new terminal
- Driver not connected / External Control not running
- `joint_trajectory_controller` not active
- Trying to execute before planning (no stored plan)

---

