docker run --name=ntp            \
              --volume="/etc/timezone:/etc/timezone:ro" \
              --network docker_hadabot_network \
              --restart=always      \
              --detach              \
              --publish=123:123/udp \
              --cap-add=SYS_TIME    \
              cturra/ntp
