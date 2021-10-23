FROM ubuntu:focal

RUN apt update && apt install -y git locales python curl wget
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8

RUN apt install -y gnupg2 lsb-release
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install prerequisites
RUN export DEBIAN_FRONTEND=noninteractive && apt update && apt install -y \
  build-essential \
  cmake \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  libpython3-dev \
  cppcheck

RUN rosdep init
RUN rosdep update

# Get ROS2 latest package
ENV ROS2_WS=/root
WORKDIR $ROS2_WS

RUN wget https://ci.ros2.org/view/packaging/job/packaging_linux/lastSuccessfulBuild/artifact/ws/ros2-package-linux-x86_64.tar.bz2 \
    && tar xf ros2-package-linux-x86_64.tar.bz2

# [Ubuntu 18.04]
RUN rosdep install --from-paths $ROS2_WS/ros2-linux/share --ignore-src --rosdistro foxy -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 libopensplice69 osrf_testing_tools_cpp poco_vendor rmw_connext_cpp rosidl_typesupport_connext_c rosidl_typesupport_connext_cpp rti-connext-dds-5.3.1 tinyxml_vendor tinyxml2_vendor urdfdom urdfdom_headers"

RUN echo "source $ROS2_WS/ros2-linux/local_setup.bash" >> $HOME/.bashrc

# Install nvm, Node.js and node-gyp
ENV NODE_VERSION v10.16.1

RUN apt update && apt install -y nodejs npm

#RUN curl -sL https://raw.githubusercontent.com/creationix/nvm/v0.33.8/install.sh | bash \ 
#    && . $HOME/.nvm/nvm.sh \
#    && nvm install $NODE_VERSION && nvm alias default $NODE_VERSION

ENV PATH /bin/versions/node/$NODE_VERSION/bin:$PATH

RUN python3 -m pip install jupyter jupyterlab bqplot pyyaml ipywidgets opencv-python \ 
    && jupyter nbextension enable --py widgetsnbextension \
    && jupyter labextension install @jupyter-widgets/jupyterlab-manager 
    #&& jupyter nbextension enable --py --sys-prefix jupyros 
    #&& jupyter labextension install jupyter-ros

RUN rm ros2-package-linux-x86_64.tar.bz2

RUN git clone https://github.com/zmk5/jupyter-ros2.git

RUN curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash - 

# need to copy to jupyros/static/ as pip asks for this path !? Don't know where it gets this path from...
RUN cd jupyter-ros2 \
    && mkdir -p jupyros/static && cp js/lib/* jupyros/static/ \ 
    && python3 -m pip install -e .

# RUN jupyter nbextension install --py --symlink --sys-prefix jupyros \
#     && jupyter nbextension enable --py --sys-prefix jupyros

ENTRYPOINT ["jupyter", "lab", "--ip=0.0.0.0", "--no-browser", "--allow-root"]
