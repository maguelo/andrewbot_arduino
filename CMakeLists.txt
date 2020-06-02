cmake_minimum_required(VERSION 3.10)
project(MeccanoidAPI)

set(CMAKE_CXX_STANDARD 11)

IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  #ADD_DEFINITIONS(-DSYSTEMTYPE_LINUX)
  ADD_DEFINITIONS(-DTESTING)
	# SET system folder name
  SET(SYSTEM_FOLDER_NAME linux )
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

# SET location for binary output and lib
SET(SRC_PATH ${CMAKE_CURRENT_SOURCE_DIR}/src)
SET(LIB_PATH ${CMAKE_CURRENT_SOURCE_DIR}/out/lib)
SET(EXECUTABLE_OUTPUT_PATH ${OUT_PATH})
SET(LIBRARY_OUTPUT_PATH ${LIB_PATH})

INCLUDE_DIRECTORIES(src)

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(test) 
ADD_SUBDIRECTORY(lib/googletest)

# include_directories(src)
# include_directories(src/stubs)
# #include_directories(lib/Arduino-Stub/src)
# #include_directories(lib/MeccanoidForArduino)

# add_subdirectory(src)
# add_subdirectory(src/stubs)
# add_subdirectory(test)

# add_subdirectory(lib/googletest)
# #add_subdirectory(lib/Arduino-Stub)
# #add_subdirectory(lib/arduino-mock)





