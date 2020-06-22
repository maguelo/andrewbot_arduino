#!/bin/bash
TARGET='arduino_lib'
ARDUINO_LIBRARIES=~/sketchbook/libraries

if [ ! -d "$TARGET" ]; then
    mkdir "$TARGET"    
fi

echo "Generate Meccanoid.zip"
zip  "$TARGET"/Meccanoid.zip stubs/Meccanoid.h stubs/Meccanoid.cpp


if [ -d "$ARDUINO_LIBRARIES" ]; then
    echo "Copy Meccanoid library to arduino libraries"
    TARGET_MECCANOID="$ARDUINO_LIBRARIES"/Meccanoid
    if [ ! -d "$TARGET_MECCANOID" ]; then 
        mkdir "$TARGET_MECCANOID"
    fi
    cp stubs/Meccanoid.h stubs/Meccanoid.cpp "$TARGET_MECCANOID"
fi


echo "Generate RobotBrain"
zip "$TARGET"/RobotBrain.zip src/RobotBrain.h src/RobotBrain.cpp

if [ -d "$ARDUINO_LIBRARIES" ]; then
    echo "Copy RobotBrain library to arduino libraries"
    TARGET_ROBOTBRAIN="$ARDUINO_LIBRARIES"/RobotBrain
    if [ ! -d "$TARGET_ROBOTBRAIN" ]; then 
        mkdir "$TARGET_ROBOTBRAIN"
    fi
    cp src/RobotBrain.h src/RobotBrain.cpp "$TARGET_ROBOTBRAIN"
fi

