#/bin/bash
docker run -it --rm --volume="/etc/timezone:/etc/timezone:ro" -v  $(pwd):$(pwd) --workdir $(pwd) rosplanning/navigation2:main.release