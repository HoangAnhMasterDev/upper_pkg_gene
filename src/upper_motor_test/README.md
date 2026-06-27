# upper_motor_test

Safe test command publisher for the upper-body motor bridge.

It subscribes:

- `/upper_motor/state` from `upper_motor_bridge`
- `/upper_test/enable` (`std_msgs/Bool`)
- `/upper_test/mode` (`std_msgs/String`): `hold`, `single_sine`, `sine`

It publishes:

- `/upper_motor/command` (`upper_motor_bridge/msg/UpperMotorCommand`)

Default behavior is inactive until enabled. When enabled, it captures the current upper-body posture and holds it. In sine modes, it commands a small sine around the captured posture.

## Basic usage

```bash
ros2 launch upper_motor_test upper_motor_test.launch.py
ros2 topic pub --once /upper_test/enable std_msgs/msg/Bool "{data: true}"
```

Disable:

```bash
ros2 topic pub --once /upper_test/enable std_msgs/msg/Bool "{data: false}"
```

Single-joint sine test:

```bash
ros2 param set /upper_motor_test_node mode single_sine
ros2 param set /upper_motor_test_node test_joint_index 0
ros2 param set /upper_motor_test_node amplitude_rad 0.02
ros2 topic pub --once /upper_test/enable std_msgs/msg/Bool "{data: true}"
```
