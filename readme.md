### Generate libraries
rosrun rosserial_arduino make_libraries.py .  

## rosserial

### python version
rosrun rosserial_python serial_node.py /dev/ttyACM0

### C++ version (better performance)
roslaunch rosserial_server serial.launch port:=/dev/ttyACM0




https://raymii.org/s/tutorials/Cpp_project_setup_with_cmake_and_unit_tests.html
