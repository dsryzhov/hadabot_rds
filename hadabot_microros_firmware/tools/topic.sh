#/bin/bash
docker run -it --rm --volume="/etc/timezone:/etc/timezone:ro" -v  $(pwd):$(pwd) --workdir $(pwd) --env TOPIC_PARAMS="$@" ros:foxy /bin/bash -c "source /opt/ros/foxy/setup.bash && echo Running command ros2 topic $TOPIC_PARAMS && ros2 topic $TOPIC_PARAMS" \
