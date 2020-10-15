SET SCRIPT_DIR=%~dp0

docker run -it -v rds/esp-idf-microros  %SCRIPT_DIR%:/root/hadabot_microros_firmware /bin/bash -c "cd /root/hadabot_microros_firmware && idf.py build"