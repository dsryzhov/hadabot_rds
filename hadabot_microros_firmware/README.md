# Micro-ROS based firmware for HADABOT 
see https://micro-ros.github.io/
see www.hadabot.com

Original firmware for Hadabot is based on MicroPython and communicates with ROS system over websocket.

Micro-ROS based firmware are built using ESP IDF SDK (C++).
Communication with ROS is based on UDP protocol. 
For hardware control high level arduino-esp32 API is used.

Micro-ROS firmware communicates with ROS2 over the same topics as original firmware:
- publish message on topic /hadabot/log/info every 5 seconds
- subscribes to topics /hadabot/wheel_power_left and /hadabot/wheel_power_right messages and controls motors based on receiving this messages
- asynchronously (to measurements) publish topics /hadabot/wheel_radps_left and hadabot/wheel_radps_right every 15 ms (time period can be configured)

Main feature of this Hadabot firmware :
- there is no control command lattency 
- wheel angular velocities are measured for every sensor signal level change by default (sensor calibration is neeeded)
- time period for wheels angular velocity measurements publication is 15 ms (can be configured). 

## Micro-ROS agent

This Hadabot firmware is based on Micro-ROS project.
It enables esp32 application to communicate with a ROS2 system.

To make it possible Micro-ROS Agent should be started on the host.
It will communicate with esp32 application via WiFi using UDP protocol
and bridge it to a ROS2 system.

To start Micro-ROS agent run script

```bash
./agent.sh
```

## Docker container with ESP IDF SDK 4.0 and all build dependencies 

To build this firmware a docker image dsryzhov/esp-idf-microros:idf-v4.0-arduino is used.
It contains preinstalled ESP IDF SDK v4.0, ESP-IDF components: Micro-ROS and arduino-esp32 
and their build dependencies.

Dockerfile for this image is accessible in the project directory docker\esp-idf-microros-arduino.

Your don't need to build\pull this image by your self.
This docker image will be pulled up automatically from hub.docker.com
when first running idf.sh scripts in the project directory.

When the docker container is run current project directory 
is mapped to container directory with the same path.
As a result all created files will be available at the host after command run.

To use external ESP IDF components (Micro-ROS and arduino-32) 
in the project CMakeLists.txt EXTRA_COMPONENT_DIRS variable is defined 
with the path to components dir in the docker container.

set (EXTRA_COMPONENT_DIRS "~/ext_components")

## Configure firmware

Before firmware build it's configuration is needed.
MicroROS IP/port, WiFi credentials and motor\sensor pins are needed to define.

To configure hadabot microros firmware run script 

```bash
./idf.sh menuconfig
```

The textual menu will appear. 
Using this textual menu you can set all the parameters
1. set your microros agent IP:port and your wifi credentials 	
> micro-ROS Settings
   -> micro-ROS Agent IP (IP addrest of your host with execution microros agent )
   -> micro-ROS Agent Port (8888)
   -> Wi-Fi configuration 
      -> WiFi SSID
	  -> WiFi Password

2. set pin numbers for motors and sensors
> Hadabot firmware configuration

After configuration ends Exit textual menu and save new configuration at the end.
   
This configuration is saved in sdkconfig file in the project directory.
This file is not stored in git repo (added to .gitignore).


## Build firmware

To build the Hadabot Microros firmware run script

```bash
./idf.sh build
```

## Flash firmware

If you use docker on Windows 10 (based on WSL2) (as I am) you need to flush esp32 from Windows host
running command.

In this case to flash firmware your need esptool installed on your Windows host.
To flush program simply run

```bash
./flash.bat
```

With WSL2 configuration there is  a probem to flush program from docker container.
By default containers on WSL2 does not see serial ports yet.

If you use containers on Linux host or Windows width WSL1 you can 
flash program to ESP from container. Simply run command in docker container

```bash
./idf.sh flash
```

## Check connection

On Linux host run ./idf.sh monitor

In the monitor output you need to check that esp32 is connected to WiFi AP and 
communication with MicroROS agent was established. 

Also check in the Micro-ROS agent console that communication is established and messages are receiving.

## Check publish\subscribe with ros2 messages 

1. Check hadabot hearbeat messages are received
ros2 topic echo /hadabot/log/info

2. Subscribe to messages whith wheels angular velocities

Run the following commands in two different terminals

ros2 topic echo /hadabot/wheel_radps_left
ros2 topic echo /hadabot/log/wheel_radps_right

3. Run forward, backward and stop commands for left motor

Run the following commands in one terminal in sequence and check left motor rotation.

ros2 topic pub --once /hadabot/wheel_power_left  std_msgs/msg/Float32 "{data: 1.0}"
ros2 topic pub --once /hadabot/wheel_power_left  std_msgs/msg/Float32 "{data: -1.0}"
ros2 topic pub --once /hadabot/wheel_power_left  std_msgs/msg/Float32 "{data: 0.0}"

4. Run forward, backward and stop right motor

Run the following commands in one terminal in sequence and check right motor rotation.

ros2 topic pub --once /hadabot/wheel_power_right  std_msgs/msg/Float32 "{data: 1.0}"
ros2 topic pub --once /hadabot/wheel_power_right  std_msgs/msg/Float32 "{data: -1.0}"
ros2 topic pub --once /hadabot/wheel_power_right  std_msgs/msg/Float32 "{data: 0.0}"

5. Check that messages with wheels angular velocities were received 

Check two terminals with message subscriptions made before 
and validate that messages were received. 



