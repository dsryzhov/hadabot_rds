#/bin/bash
docker run -it --rm --network=docker_hadabot_network --volume="/etc/timezone:/etc/timezone:ro" -v  $(pwd)/..:$(pwd)/.. --workdir $(pwd)/.. rosplanning/navigation2:main.release \
