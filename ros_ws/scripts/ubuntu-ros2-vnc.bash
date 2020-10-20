#/bin/bash

docker run -d -p 6080:80 -p 5900:5900 -e RESOLUTION=1920x1080 --shm-size="2g" -v $(pwd):$(pwd) --workdir $(pwd) dsryzhov/ubuntu-ros2-vnc-desktop