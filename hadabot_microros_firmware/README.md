# Micro-ROS based firmware for HADABOT 
see https://micro-ros.github.io/
see www.hadabot.com

Original firmware for Hadabot is based on MicroPython and communicates with ROS system over websocket.
This Micro-ROS based firmware are built using ESP IDF SDK (C++). Communication with ROS is based on UDP protocol. 
For hardware control high level arduino-esp32 API is used.

This firmware can be used for implementing two control strategy:
- local position and motion control with goal position received from ROS2
- ROS2 navigation control with linear and angular velocity received from ROS2 navigation (via /cmd_vel topic)

By default the first strategy is used in code.
For the second strategy to be used some modification in code is needed. (Position Controler switch off )
In the future control strategy change will be implemented by ROS2 command. 

Local position and motion control strategy does not use ROS2 navigation stack at all.
Hadabot receives goal position and plan and implements the trajectory to this position by itself.
This strategy was implemented to debug Hadabot motion control algorithms with neglecting communication (latency) problems when 
communicating with PC. 

The second control strategy (with ROS2 navigation) showed nearly the same very good results in trajectory following. 
Problems with comminication latency were solved by additional design solutions.

For ROS2 navigation scenario the main difference in PC and ESP32 roles are listed bellow:
- time is sycnronised between PC and ESP32
- position estimation (position, orientation, linear and angular velocities) is done on ESP32 
- position measurements are sent to ROS2 topic with timestamp
- ESP32 subscribes to target linear and angular velocities (not motor's power commands)
- motion control is implemented on ESP32 (based on target linear and angular velocities) but not in ROS2 node on PC

Another difference with measurements and control algorithms in the firmware
- wheel angular velocities are measured for every sensor signal level change (sensor calibration is neeeded)
- MPU6050 orientation measuremens are used for hadabot orientation and angular velocity estimation 
(estimations based on wheels angular velosities are not used at all for position estimation)
- PID regulator are used for motors controls

These design solutions results in very small control command lattency and very accurate trajectory following by Hadabot.

Below is the function decomposition of the firmware:

Initialization functions:
- ros2 communication initialization
- time synchronization with ROS2

Communication functions:
- receive goal position from ROS2
- receive linear and angular velosity from ROS 2
- publish log messages
- publish timestamped odom2 messages (every 20ms)
- publish distance to forward obstacles

Detection functions:
- measure left wheel angular velocity
- measure right wheel angular velocity
- measure hadabot orientation (using MPU6050 sensor)
- measure hadabot angular velocity (using MPU6050 sensor)
- measure distance to forward obstacles 
 
Control functions:
- estimate current position
- control position (used only in fully local control straegy)
- control motor motion (two PID regulators)

Main scenario for local position and motion control strategy with position goal received from ROS:
- goal position is received from ROS2
- hadabot' orintation and angular velocity  is estimated based on MPU6050 measurements
- rotation sensors measure wheels angular velocities
- local position estimator calculates hadabot linear velocity based on wheel's angular velocities
- local position estimator gets hadabot orientation and angular velocity from MPU6050 measurements
- local position estimator calculates current hadabot position and orientation based on calculated hadabot linear and angular velocities
- local position controler compare current position with goal position
- local position controler calculates new target linear and angular velocities of the hadabot (using PID regulator)
- local motion controler calculates target left and right wheels power values based on calculated hadabot target linear and angular velocities
- local motion contoroler calculates new current motor's power values (using PID regulatro) (and updates them every 20 ms)
- motor controller set new current motor power values
- position estimation is published to ros2 topic every 20ms (asyncronously)

Main scenarion for ROS2 navigation:
- target linear and angular velocity are received from ROS
- goal position is received from ROS2
- hadabot' orintation and angular velocity  is estimated based on MPU6050 measurements
- rotation sensors measure wheels angular velocities
- local position estimator calculates hadabot linear velocity based on wheel's angular velocities
- local position estimator gets hadabot orientation and angular velocity from MPU6050 measurements
- local position estimator calculates current hadabot position and orientation based on calculated hadabot linear and angular velocities
- local motion controler calculates target left and right wheels power values based on received from ROS2 hadabot target linear and angular velocities
- local motion contoroler calculates new current motor's power values (using PID regulatro) (and updates them every 20 ms)
- motor controller set new current motor power values
- position estimation is published to ros2 topic every 20ms (asyncronously)


Micro-ROS firmware communitcates to ROS2 with the following  topics:
- publish message on topic /hadabot/log/info every 5 seconds
- subscribes to topics /cmd_vel and /hadabot/cmd_vel (can be uses any of these topics, reaction is the same)
- subscribes to topics /hadabot/goal_pose
- asynchronously (to measurements) publish topics hadabot/odom2d every 20 ms (time period can be configured)

To neglect communication latency with ROS in ROS2 navigation scenario timestamped measurements are used
- when initializing Micro-ROS time syncronization with ROS2 system is done
- rotation sensors calculates angular velocities on EACH signal front change (not statistically, for example every 100ms)
- rotation sensors timestamp their wheels angular velocities mesurements 
- position estimator timestamps it's position estimation based on angular velocities mesurements timestamps
- timestamps of current position estimation are sent to ROS topic odom2
- timestamps of current position estimation are used in ROS2 navigation algorithms

The following FreeRTOS tasks are used on ESP32:
- ROS2 communication task 
- motion controller task calculates and updates current motor power values
- position controller task calculates and updates target linear and angular velocities of the hadabot
- left wheel rotation sensor task (hadle events from left sensor IRQ)
- right wheel rotation sensor task (hadle events from left sensor IRQ)

The following interrupts handlers are used:
- left rotation sensor IRQ hadler
- right rotation sensor IRQ hadler


There is no additional task for position estimation. Position estimation is called from rotation sensors tasks after each wheel angular velocity measurement.

Main hadabot class HadabotHW is implemented in hadabot_hw.cpp
https://github.com/dsryzhov/hadabot_rds/tree/master/hadabot_microros_firmware/main

Other classes are implemented as components here
https://github.com/dsryzhov/hadabot_rds/tree/master/hadabot_microros_firmware/components/rds/libraries

ROS2 communicaton is implemented here
https://github.com/dsryzhov/hadabot_rds/blob/master/hadabot_microros_firmware/main/microros_interface.cpp

Wheel angular velocity estimation is implemented here
https://github.com/dsryzhov/hadabot_rds/blob/master/hadabot_microros_firmware/components/rds/libraries/RotSensor/src/rotsensor.cpp

Position estimation is implemented here (MPU6050 measurements are used for orientation estimation)
https://github.com/dsryzhov/hadabot_rds/blob/master/hadabot_microros_firmware/components/rds/libraries/PosEstimator/src/PosEstimator.cpp

Several position control algotithms are implemented (currently PosPurePursuitController is used in HadabotHW)
https://github.com/dsryzhov/hadabot_rds/tree/master/hadabot_microros_firmware/components/rds/libraries/PosController

Motion control adjust motor angular velocities based on difference between  target velocities and measures angular velocities of wheels 
https://github.com/dsryzhov/hadabot_rds/blob/master/hadabot_microros_firmware/components/rds/libraries/MotionController/MotionController.cpp


Motor control is implemented here
https://github.com/dsryzhov/hadabot_rds/blob/master/hadabot_microros_firmware/components/rds/libraries/Motor/src/motor.cpp

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

```bash
com2tcp-rfc2217.bat COM3 3333
```

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

## Check communication with Micro-ROS agent

After monitor is starte in the monitor output  check that esp32 is connected to WiFi AP and 
communication with MicroROS agent is established. 

Also check in the Micro-ROS agent console (run via agent.sh script at the beginning)
that communication is established and messages are receiving.

## Send\Receive ROS2 messages to esp32

After that try to subscribe and publish ROS2 messages for Hadabot.

1. Check hadabot hearbeat messages are received
```bash
ros2 topic echo /hadabot/log/info
```


2. Subscribe to messages whith position odometry

Run the following commands in two different terminals

```bash
ros2 topic echo /hadabot/odom2
```


3. Run the following command to set goal position for local position control scenario

```bash
ros2 topic pub --once /hadabot/goal_pose geometry_msgs/msg/Pose2D "{x: 0.0, y: 0.0, theta: 1.57}"
```

4. Run the following command to set target linear and angular velocity for

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 7.85}}"
```

## Run Hadabot ROS2 nodes 

You can control Hadabot via Web GUI 

For that you need to start additional ROS nodes via docker containers.

Before running next command 
stop Micro-ROS agent started via aget.sh script

To start all needed ROS2 nodes run in hadabot_rds/ros_ws/docker command 

```bash
docker-compose up
```

It will start three docker containers
- ros2-webbridge
- ros2-microrosagent
- ros2-Hadabot

After containers are started you can control Hadabot from the page
https://www.hadabot.com/tools/teleop.html


After you are finishged stop Hadabot containers using command

```bash
docker-compose down
```
