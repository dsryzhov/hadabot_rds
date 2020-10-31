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

To start only Micro-ROS agent (without) Hadabot ros nodes you can run script

```bash
./agent.sh
```

## Docker container with ESP IDF SDK 4.0 and all build dependencies 

To build this firmware a docker image dsryzhov/esp-idf-microros:idf-v4.0-arduino is used.
It contains preinstalled ESP IDF SDK v4.0, Micro-ROS build dependencies and arduino-esp32 ESP-IDF componet.

Dockerfile for this image is accessible in the project directory docker\esp-idf-microros-v4.0-arduino.
It's based on another docker image dsryzhov/esp-idf-microros:idf-v4.0 (see Dockerfile in docker\esp-idf-microros-v4.0)
that does not contains arduino-esp32 component. 

Your don't need to build\pull this images by your self.
This docker images will be pulled up automatically from hub.docker.com
when first running runidf.sh scripts in the project directory.

When the docker container is run current project directory 
is mapped to container directory with the same path.
As a result all created files will be available at the host after commands run in the docker container.

To use external ESP IDF components (Micro-ROS and arduino-32) 
in the project CMakeLists.txt EXTRA_COMPONENT_DIRS variable is defined 
with the path to components dir in the docker container.

set (EXTRA_COMPONENT_DIRS "~/esp32/ext_components")

## Run docker container with ESP IDF

To run docker container use script

```bash
./runidf.sh
```

A docker container with ESP IDF SDK 4.0 will start in interactive mode. 
After that you will be able to run idf.py commands in your terminal inside container. 
All next commands should be run in this container. 

## Configure firmware

Before firmware build it's configuration is needed.
MicroROS IP/port, WiFi credentials and motor\sensor pins are needed to define.

To configure hadabot microros firmware run command inside docker container

```bash
./idf.py menuconfig
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


## Define Micro-ROS agent IP address in colcon.meta (one more time)

Currently Micro-ROS agent IP address configuration via menuconfig does not work.
You need to edit components/Micro-ROS/colcon.meta 
and change IP address to your machine with Micro-ROS agent

"-DRMW_UXRCE_DEFAULT_UDP_IP=192.168.1.64",


## Preruquisites for Windows 10 docker containers

If you use docker on Windows 10 (based on WSL2) (as I am) to be able to run flash and monitor commands 
from docker container you need to do additional 

Linux distros installed on WSL2 (or inside containers) does not see COM ports yet. 
(Microsoft working on solving this now).

To be able to flash\monitor from docker container you need to map Windows COM port (with ESP32 plugged in)
to TCP port that can be used from docker container. 

I use utility hub4com
https://sourceforge.net/projects/com0com/

you can run it in Windows with COM port and TCP port defined
com2tcp-rfc2217.bat COM3 3333

after that you can use idf.py commands flash\monitor inside container adding TCP port information.

## Build / flash / monitor  firmware

To build / flash / monitor the Hadabot Microros firmware run command inside docker container

```bash
./idf.py build flash monitor 
```

When flash will starting to connect you may need to press BOOT button on ESP32 board until connection is established. 

For docker container based on WSL2 use command inside container (see Windows 10 prerequisites in the previous section)

```bash
./idf.py build flash monitor -p rfc2217://192.168.1.64:3333
```
use your host IP.

## Check communication 

After monitor is starte in the monitor output  check that esp32 is connected to WiFi AP and 
communication with MicroROS agent is established. 

Also check in the Micro-ROS agent console (run via agent.sh script at the beginning)
that communication is established and messages are receiving.

## Send\Receive ROS2 messages to esp32

After that try to subscribe and publish ROS2 messages for Hadabot.

1. Check hadabot hearbeat messages are received
ros2 topic echo /hadabot/log/info

2. Subscribe to messages whith wheels angular velocities

Run the following commands in two different terminals

ros2 topic echo /hadabot/wheel_radps_left/temperature
ros2 topic echo /hadabot/log/wheel_radps_right/temperature

Temperature message type is used to publish timestamped data from the firmware temporarily.
In the future it will be changed to custom ROS message.

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



