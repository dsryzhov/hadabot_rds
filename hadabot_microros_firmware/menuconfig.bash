#/bin/bash
docker run -it --rm -v  $(pwd):$(pwd) --workdir $(pwd) rds/esp-idf-microros /bin/bash -c 'pip3 install catkin_pkg lark-parser empy && idf.py menuconfig &&'
