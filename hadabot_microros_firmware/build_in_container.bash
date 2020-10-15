#/bin/bash
docker run -it --rm -v  $(pwd):$(pwd) --workdir $(pwd) dsryzhov/esp-idf-microros /bin/bash -c 'pip3 install catkin_pkg lark-parser empy && idf.py build &&'
