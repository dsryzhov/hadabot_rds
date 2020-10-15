#/bin/bash

docker run --rm -it  -v $(pwd):$(pwd) --workdir $(pwd) ros:foxy /bin/bash -c 'source /opt/ros/foxy/setup.bash && colcon build' 
