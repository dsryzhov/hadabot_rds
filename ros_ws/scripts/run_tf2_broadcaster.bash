#/bin/bash

docker run -it --rm  -v $(pwd):$(pwd) --workdir $(pwd) --name  hadabot_tf2 ros:foxy /bin/bash -c 'source /opt/ros/foxy/setup.bash && source ./install/local_setup.bash && ros2 run hadabot_tf2 hadabot_tf2_broadcaster'
