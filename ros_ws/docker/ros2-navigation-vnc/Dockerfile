FROM dsryzhov/ubuntu-ros2-vnc-desktop:latest

RUN apt update -q && apt install -yq ros-foxy-navigation2 ros-foxy-nav2-bringup nano && rm -rf /var/lib/apt/lists/*

CMD [ "/bin/bash" ]
