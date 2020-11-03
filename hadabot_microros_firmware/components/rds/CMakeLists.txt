set(LIBRARY_SRCS
  libraries/Motor/src/motor.cpp
  libraries/RotSensor/src/rotsensor.cpp
  libraries/HC-SR04/src/hcsr04.cpp 
  libraries/PosEstimator/src/PosEstimator.cpp
  libraries/TinyMPU6050/src/TinyMPU6050.cpp   
  libraries/MPU6050/MPU6050.cpp   
  libraries/I2Cdev/I2Cdev.cpp
  )
  
set(COMPONENT_SRCS ${LIBRARY_SRCS})  

set(COMPONENT_ADD_INCLUDEDIRS
  libraries/Motor/src
  libraries/RotSensor/src
  libraries/HC-SR04/src
  libraries/PosEstimator/src
  libraries/TinyMPU6050/src
  libraries/MPU6050
  libraries/I2Cdev
  )

set(COMPONENT_REQUIRES arduino)
register_component()
  
