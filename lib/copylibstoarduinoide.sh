#!/bin/sh

mkdir -p /c/Users/steve/Documents/Arduino/libraries/LS7366
cp ArduinoLibs/LS7366/LS7366/LS7366R.cpp /c/Users/steve/Documents/Arduino/libraries/LS7366/
cp ArduinoLibs/LS7366/LS7366/LS7366R.h /c/Users/steve/Documents/Arduino/libraries/LS7366/

mkdir -p /c/Users/steve/Documents/Arduino/libraries/MPU9250/
cp ArduinoLibs/MPU-9250/Libraries/Arduino/src/MPU9250.cpp /c/Users/steve/Documents/Arduino/libraries/MPU9250
cp ArduinoLibs/MPU-9250/Libraries/Arduino/src/MPU9250.h /c/Users/steve/Documents/Arduino/libraries/MPU9250

mkdir -p /c/Users/steve/Documents/Arduino/libraries/quaternionFilters/
cp ArduinoLibs/MPU-9250/Libraries/Arduino/src/quaternionFilters.cpp /c/Users/steve/Documents/Arduino/libraries/quaternionFilters
cp ArduinoLibs/MPU-9250/Libraries/Arduino/src/quaternionFilters.h /c/Users/steve/Documents/Arduino/libraries/quaternionFilters

mkdir -p /c/Users/steve/Documents/Arduino/libraries/SonarArray
cp ArduinoLibs/SonarArray/SonarArray/SonarArray.cpp /c/Users/steve/Documents/Arduino/libraries/SonarArray/
cp ArduinoLibs/SonarArray/SonarArray/SonarArray.h /c/Users/steve/Documents/Arduino/libraries/SonarArray/

mkdir -p /c/Users/steve/Documents/Arduino/libraries/ros_lib/src
rm -rf /c/Users/steve/Documents/Arduino/libraries/ros_lib/src/*.*
cp -R ArduinoLibs/ros_lib/* /c/Users/steve/Documents/Arduino/libraries/ros_lib/src
mv /c/Users/steve/Documents/Arduino/libraries/ros_lib/src/library.properties /c/Users/steve/Documents/Arduino/libraries/ros_lib/

mkdir -p /c/Users/steve/Documents/Arduino/libraries/avc_common/
cp common/ros_topics.h /c/Users/steve/Documents/Arduino/libraries/avc_common/
cp common/node_names.h /c/Users/steve/Documents/Arduino/libraries/avc_common/

