#/bin/bash
docker run -it --rm --user espidf --volume="/etc/timezone:/etc/timezone:ro" -v  $(pwd):$(pwd) -v  /dev:/dev --workdir $(pwd) --env IDF_CMD=$1 dsryzhov/esp-idf-microros:idf-v4.0-arduino /bin/bash -c 'echo Running command idf.py $IDF_CMD && idf.py $IDF_CMD ' 