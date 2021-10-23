#/bin/bash
docker run --network=docker_hadabot_network --rm -p 6080:80 -p 5900:5900 -e RESOLUTION=1920x1080 --shm-size="2g" --volume="/etc/timezone:/etc/timezone:ro" -v $(pwd)/../..:$(pwd)/../.. dsryzhov/ros2-navigation-vnc-desktop:latest \
