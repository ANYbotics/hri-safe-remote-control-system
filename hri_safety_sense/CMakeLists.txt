cmake_minimum_required(VERSION 2.4.6)
project(hri_safety_sense)

find_package(catkin REQUIRED COMPONENTS
message_generation
roscpp
)


## Generate services in the 'srv' folder
add_service_files(
  FILES
  EmergencyStop.srv
  KeyString.srv
  KeyValue.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
)

catkin_package()

include_directories(include)
include_directories(src)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(safe_remote_control 
src/main.cpp 
src/VscProcess.cpp 
src/VehicleInterface.c 
src/JoystickHandler.cpp 
src/SerialInterface.c)

add_dependencies(safe_remote_control hri_safety_sense_generate_messages_cpp)

target_link_libraries(safe_remote_control ${catkin_LIBRARIES})
