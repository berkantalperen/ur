# URSim + ROS2 Driver Baseline (UR5e)

This document describes a **known-good baseline** setup for connecting **URSim (e‑Series)** with the **ROS2 Universal Robots Driver** using **External Control**, running on **Docker + WSL (Ubuntu)**.

---

## System Assumptions

- Robot: **UR5e**
- URSim: `universalrobots/ursim_e-series` Docker image
- ROS2 driver running on **WSL Ubuntu (host)**
- Networking: Docker **default bridge**
- Control via **External Control URCap**
- Reverse interface port: **50002**

---

## Key Networking Rule (IMPORTANT)

External Control always connects **robot → driver**.

In Docker bridge networking:

- `172.17.0.x` → Docker containers  
- `172.17.0.2` → **URSim container itself**
- `172.17.0.1` → **Docker/WSL host (your ROS2 driver)**

### ✅ Correct
External Control **Remote Host** = `172.17.0.1`

### ❌ Wrong
External Control **Remote Host** = `172.17.0.2`  
(this makes URSim try to connect to itself → connection refused)

---

## Connectivity Sanity Check

From the **URSim container**, test whether the driver is reachable:

```bash
docker exec -it ursim_ur5e bash -lc 'timeout 2 bash -lc "</dev/tcp/172.17.0.1/50002" && echo OK || echo FAIL'
```

Expected output:
```text
OK
```

If this fails, the ROS2 driver is not listening or the IP/port is wrong.

---

## External Control Configuration (PolyScope)

In URSim PolyScope:

1. Installation → URCaps → External Control
2. Set:
   - **Remote Host / Remote PC**: `172.17.0.1`
   - **Port**: `50002`
3. Save the installation
4. Load & run the External Control program

---

## Controller State (Reference)

Typical healthy state:

```bash
ros2 control list_controllers
```

Expected:
- `scaled_joint_trajectory_controller` → **active**
- `joint_state_broadcaster` → **active**
- `io_and_status_controller` → **active**

---

## Status Topics

Available IO & status topics:

```bash
ros2 topic list -t | grep io_and_status_controller
```

Example:
- `/io_and_status_controller/robot_program_running [std_msgs/msg/Bool]`
- `/io_and_status_controller/robot_mode`
- `/io_and_status_controller/safety_mode`

### Echoing program state (IMPORTANT)

Do **not** use `--once` initially (race condition):

```bash
ros2 topic echo /io_and_status_controller/robot_program_running
```

Start/stop the External Control program and observe:
```text
data: true
data: false
```

If `--once` is used, the publisher may not be discovered in time.

---

## Common Failure Modes

### Connection Refused
- Remote Host set to URSim container IP (`172.17.0.x`)
- Driver not running or not listening on 50002

### `ros2 topic echo` says “not published yet”
- Topic exists but publisher hasn’t emitted yet
- Using `--once` on event‑driven topics

Fix: echo without `--once` or trigger a state change.

---

## When to Make Networking Static (Optional)

For long‑term or shared setups:

- Use a **user-defined Docker network**
- Run URSim + driver containers on the same network
- Use **container names** instead of IPs

For local development, **`172.17.0.1` is perfectly fine and stable**.

---

## Baseline Status

✅ URSim running  
✅ External Control connected  
✅ ROS2 driver reachable  
✅ Controllers loaded  
✅ Status topics visible  

This setup is the **base reference state** for all future workspaces and control experiments.
