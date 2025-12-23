# ur_imu_teleop_ws

IMU-driven teleoperation helpers for a UR5e using **MoveIt Servo**.

This workspace focuses on **wrist orientation control** using **quaternions** to avoid gimbal lock. A simulated IMU publisher is included so you can test without hardware.

## Packages

- `ur_imu_teleop`
  - `imu_sim_node`: publishes simulated `sensor_msgs/Imu` messages
  - `imu_servo_node`: consumes IMU orientation and publishes `geometry_msgs/TwistStamped` to MoveIt Servo

## Key behaviors

- Uses **quaternion error** + proportional gain to compute angular velocity commands.
- Commands are **clamped** by `max_angular_velocity` (physical limit).
- Node only uses the **latest IMU orientation**, so it does not follow old paths.

## What to start first (prereqs)

1. Start the UR driver (real robot or URSim).
2. Start MoveIt Servo for the UR5e (so it listens on `/servo_node/delta_twist_cmds`).
3. Then launch the IMU teleop nodes from this workspace.

If you already have a MoveIt Servo bringup launch, make sure it publishes TF for
`base_link` → `tool0` and has the `delta_twist_cmds` input enabled.

## Build this workspace

```bash
cd /workspace/ur/ur_imu_teleop_ws
colcon build --symlink-install
source install/setup.bash
```

## Run (simulated IMU + servo bridge)

```bash
ros2 launch ur_imu_teleop imu_teleop.launch.py
```

This launch starts:
- `imu_sim_node` → publishes `sensor_msgs/Imu` on `/imu/data`
- `imu_servo_node` → publishes `geometry_msgs/TwistStamped` on `/servo_node/delta_twist_cmds`

> Ensure a MoveIt Servo node is running and listening on `/servo_node/delta_twist_cmds`.

## Swap the simulator for a real IMU

When your wearable IMU is ready, stop `imu_sim_node` and point `imu_servo_node`
at your IMU topic:

```bash
ros2 run ur_imu_teleop imu_servo_node --ros-args -p imu_topic:=/your/imu/topic
```

## Parameters

`imu_sim_node`
- `frame_id` (string): IMU frame id
- `publish_rate_hz` (float): publish rate
- `roll_amplitude_rad`, `pitch_amplitude_rad`, `yaw_amplitude_rad` (float): sinusoidal amplitudes
- `angular_speed_rad_s` (float): sinusoidal speed

`imu_servo_node`
- `imu_topic` (string): IMU topic name
- `command_topic` (string): MoveIt Servo command topic
- `base_frame` (string): base frame for TF lookup
- `ee_frame` (string): end-effector frame for TF lookup
- `publish_rate_hz` (float): command rate
- `kp` (float): proportional gain
- `max_angular_velocity` (float): physical angular velocity limit (rad/s)
- `imu_timeout_s` (float): drop IMU data older than this value
- `tf_warn_throttle_s` (float): throttle TF warning logs (seconds)

## Quick sanity checks

```bash
ros2 topic echo /imu/data --once
ros2 topic echo /servo_node/delta_twist_cmds --once
```

## Notes

- The IMU orientation is treated as a **target orientation in the base frame**. If your IMU frame differs, you should add a TF transform or pre-rotate the quaternion before publishing.
- `imu_servo_node` publishes **only angular velocity** commands for wrist orientation. Linear motion is set to zero.

## Troubleshooting

- **TF lookup failed: "base_link" passed to lookupTransform argument target_frame does not exist**
  - MoveIt/UR driver is not publishing TF yet. Make sure the UR driver + MoveIt Servo stack is running and that the frames match your robot (`base_frame` / `ee_frame` parameters).
