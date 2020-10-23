#/bin/bash
docker run -it --rm --volume="/etc/timezone:/etc/timezone:ro" -v  $(pwd):$(pwd) --workdir $(pwd) --env TOPIC_NAME=$1 ros:foxy /bin/bash -c 'source /opt/ros/foxy/setup.bash && ros2 topic echo $TOPIC_NAME' 
