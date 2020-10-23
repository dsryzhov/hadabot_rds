FROM dsryzhov/esp-idf-microros:idf-v4.0

COPY ./install_idf_components.sh /home/espidf/install_idf_components.sh

USER espidf

RUN cd /home/espidf && ./install_idf_components.sh 

RUN cd /home/espidf/esp32/ext_components/Micro-ROS/examples/int32_publisher && . /opt/esp/idf/export.sh && idf.py build && rm -rf build

ENTRYPOINT [ "/opt/esp/entrypoint.sh" ]

CMD [ "/bin/bash" ]
