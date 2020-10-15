FROM espressif/idf:release-v4.1
ENV TZ=Europe/Moscow
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
RUN apt update
RUN apt install lsb-release git
RUN export DEBIAN_FRONTEND=noninteractive
RUN sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt update
RUN apt install -y python3-colcon-common-extensions python3-pip
RUN . $IDF_PATH/export.sh
RUN pip3 install catkin_pkg lark-parser empy
