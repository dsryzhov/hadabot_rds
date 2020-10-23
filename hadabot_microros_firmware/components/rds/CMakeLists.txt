set(LIBRARY_SRCS
  libraries/Motor/src/motor.cpp
  libraries/RotSensor/src/rotsensor.cpp
  )
  
set(COMPONENT_SRCS ${LIBRARY_SRCS})  

set(COMPONENT_ADD_INCLUDEDIRS
  libraries/Motor/src
  libraries/RotSensor/src
  )

set(COMPONENT_REQUIRES arduino)
register_component()
  
