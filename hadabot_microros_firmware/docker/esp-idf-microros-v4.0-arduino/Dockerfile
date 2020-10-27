FROM dsryzhov/esp-idf-microros:idf-v4.0

COPY ./install_idf_components.sh /home/espidf/install_idf_components.sh

USER espidf

RUN cd /home/espidf && ./install_idf_components.sh 

ENTRYPOINT [ "/opt/esp/entrypoint.sh" ]

CMD [ "/bin/bash" ]
