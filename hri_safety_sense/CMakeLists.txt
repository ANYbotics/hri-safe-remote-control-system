cmake_minimum_required(VERSION 2.4.6)
project(hri_safety_sense)

find_package(catkin REQUIRED COMPONENTS
message_generation
roscpp
rospy
std_msgs
)

## Genereate messages in the 'msg' folder
add_message_files(
  FILES
  KeyValueResp.msg
  RemoteStatus.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  EmergencyStop.srv
  KeyString.srv
  KeyValue.srv
  MessageConfigure.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    message_generation
    roscpp
    rospy
    std_msgs
)

include_directories(include
                    src
                    ${catkin_INCLUDE_DIRS})


add_executable(safe_remote_control 
src/main.cpp 
src/VscProcess.cpp 
src/VehicleInterface.c 
src/JoystickHandler.cpp 
src/SerialInterface.c)

add_dependencies(safe_remote_control hri_safety_sense_generate_messages_cpp)

target_link_libraries(safe_remote_control ${catkin_LIBRARIES})
