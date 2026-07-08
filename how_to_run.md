## Pseudo run the lower body while actually run the upper hardware

Terminal 1: ros2 launch upper_motor_test whole_body_visualize.launch.py
    Visualize, could run on workstation

Terminal 2: ros2 run upper_motor_test lower_humanoid_pseudo
    Pseudo code, fake the feedback of lower body, but actually run the upper hardware

Terminal 3: roslaunch upper_motor_driver upper_motor_driver.launch
    Upper driver code

Terminal 4: ros2 run upper_motor_bridge upper_motor_bridge_node
    Bridge between ros2 upper workspace and upper driver

Terminal 5: ros2 launch launch_scripts test_all.launch.py
    Compute the lower joint command

Terminal 6: ros2 launch upper_momentum_compensator upper_momentum_compensator.launch.py
    Compute the upper joint command based on the lower joint feedback

Terminal 7: ros2 run upper_motor_test upper_data_recorder.py
    Record data for the upper body only