#/bin/bash

docker run -it --rm  -v $(pwd):$(pwd) --workdir $(pwd) --name  hadabot_info ros:foxy /bin/bash -c 'source /opt/ros/foxy/setup.bash && ros2 topic echo /hadabot/log/info'
