#/bin/bash
docker run  \
  -it \
  --network=docker_hadabot_network \
  --rm --volume="/etc/timezone:/etc/timezone:ro" \
  -v  $(pwd):$(pwd) \
  --workdir $(pwd) \
  ros:foxy \
