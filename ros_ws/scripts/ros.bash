#/bin/bash

docker run -it --rm  -v $(pwd):$(pwd) --workdir $(pwd) --name  hadabot_ros ros:foxy /bin/bash -c 'source /opt/ros/foxy/setup.bash && /bin/bash'
