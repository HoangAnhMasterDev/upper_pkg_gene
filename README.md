# upper_ros2_ws

ROS2 workspace with two packages:

1. `upper_motor_bridge`
   - Reads ROS1 upper driver shared memory `/upper_motor_state`.
   - Publishes `/upper_motor/state` and `/upper_motor/joint_states`.
   - Subscribes `/upper_motor/command` and writes `/upper_motor_command`.

2. `upper_momentum_compensator`
   - Subscribes `/upper_motor/state`.
   - Subscribes `/lower_motor/joint_states` for lower-body joint states.
   - Subscribes `/imu` optionally.
   - Publishes `/upper_motor/command`.
   - Enable with `/upper_momentum/enable` using `std_msgs/msg/Bool`.

## Build

```bash
cd ~/upper_ws
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch upper_motor_bridge upper_motor_bridge.launch.py
ros2 launch upper_momentum_compensator upper_momentum_compensator.launch.py
```

Enable controller:

```bash
ros2 topic pub --once /upper_momentum/enable std_msgs/msg/Bool "{data: true}"
```

Disable:

```bash
ros2 topic pub --once /upper_momentum/enable std_msgs/msg/Bool "{data: false}"
```

## Important

`SharedMemoryTypes.hpp` must match the ROS1 driver shared-memory structs exactly.
If the ROS1 driver uses 8 DOF, keep `NUM_UPPER_ACT = 8` here.
If field order changes in ROS1, update the ROS2 bridge struct too.
