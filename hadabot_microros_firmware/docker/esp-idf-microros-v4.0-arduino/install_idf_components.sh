#!/bin/bash

set -eu

mkdir esp32 
cd esp32

mkdir ext_components
cd ext_components

git clone --depth=1 --branch idf-release/v4.0 https://github.com/espressif/arduino-esp32.git arduino

set +u
