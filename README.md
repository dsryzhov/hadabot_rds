

* ESP32 receives cmd_vel commands from ROS2
* ESP32 publish odom2D message with position, orientation, linear and angular velocities
* Sync esp32 clock and ros nodes clock with local ntp server 
* Estimate wheels angular velocities
* Control wheels angular velocities with PID
* Estimate Hadabot orientation from gyroscope (MPU605)
* Estimate position (x,y) from wheels angular velocity integration (usin orientation from gyroscope)
* Estimate linear velocity based on wheels angular velocities estimation
* Estimate angular velocity based on wheels angular velocities estimation  and gyroscope (combined)

ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 0.
5, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"