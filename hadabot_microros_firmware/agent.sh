#!/bin/bash
docker run -it --rm -p 8888:8888/udp microros/micro-ros-agent:foxy udp4 --port 8888 -v5