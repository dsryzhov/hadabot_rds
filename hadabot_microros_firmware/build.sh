#/bin/bash
docker run -it --rm --user espidf --volume="/etc/timezone:/etc/timezone:ro" -v  $(pwd):$(pwd) --workdir $(pwd) dsryzhov/esp-idf-microros:idf-v4.0 /bin/bash  -c "idf.py build &&"


