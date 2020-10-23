#!/bin/bash

set -eu

mkdir esp32 
cd esp32

mkdir ext_components
cd ext_components

git clone https://github.com/espressif/arduino-esp32.git arduino
cd arduino
git checkout -b idf-release/v4.0 origin/idf-release/v4.0

cd ..
git clone https://github.com/dsryzhov/micro_ros_espidf_component.git Micro-ROS
cd Micro-ROS
git checkout -b feature/clear-idf-component origin/feature/clear-idf-component

set +u
