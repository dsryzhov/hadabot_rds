#/bin/bash
docker run -it --rm --user espidf --volume="/etc/timezone:/etc/timezone:ro" -v  $(pwd):$(pwd) -v  /dev:/dev --workdir $(pwd) dsryzhov/esp-idf-microros:idf-v4.0-arduino /bin/bash 