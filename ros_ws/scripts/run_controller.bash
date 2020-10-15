#/bin/bash

docker run -it --rm  -v $(pwd):$(pwd) --workdir $(pwd) --name  hadabot_controller ros:foxy /bin/bash -c 'source /opt/ros/foxy/setup.bash && source ./install/local_setup.bash && ros2 run hadabot_driver hadabot_controller'
