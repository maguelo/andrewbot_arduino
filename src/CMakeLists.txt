
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR})

set(BINARY ${CMAKE_PROJECT_NAME})

set(SOURCES 
    RobotBrain.h RobotBrain.cpp 
    ../stubs/Arduino.h 
    ../stubs/Arduino.cpp 
    ../stubs/Print.h 
    ../stubs/SPI.h 
    ../stubs/Stream.h 
    ../stubs/Wire.h 
    ../stubs/WProgram.h 
    ../stubs/Meccanoid.h 
    ../stubs/Meccanoid.cpp)



#add_executable(${BINARY}_run ${SOURCES})

add_library(${BINARY}_lib STATIC ${SOURCES})




# EXAMPLE CODE
# set(BINARY ${CMAKE_PROJECT_NAME})

# #file(GLOB_RECURSE SOURCES LIST_DIRECTORIES true *.h *.cpp)

# #set(SOURCES ${SOURCES})

# set(SOURCES main.cpp Formula.cpp Formula.h)

# add_executable(${BINARY}_run ${SOURCES})

# add_library(${BINARY}_lib STATIC ${SOURCES})
