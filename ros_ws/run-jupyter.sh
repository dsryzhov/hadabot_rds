docker run -it -p 8888:8888 --network docker_hadabot_network --entrypoint /bin/bash dsryzhov/jupyter-ros2
jupyter lab --ip=0.0.0.0 --no-browser --allow-root
