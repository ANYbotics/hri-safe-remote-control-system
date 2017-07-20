cmake_minimum_required(VERSION 2.4.6)
project(hri_safety_sense)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  any_node
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

## Genereate messages in the 'msg' folder
add_message_files(
  FILES
  EstopStatus.msg
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
  LIBRARIES hri_safety_sense
  CATKIN_DEPENDS
    any_node
    message_generation
    roscpp
    rospy
    sensor_msgs
    std_msgs
)

include_directories(
  include
  src
  ${catkin_INCLUDE_DIRS}
)

add_library(hri_safety_sense
  src/VscProcess.cpp 
  src/VehicleInterface.c 
  src/JoystickHandler.cpp 
  src/SerialInterface.c
)

add_dependencies(hri_safety_sense
  hri_safety_sense_gencpp
)

add_executable(safe_remote_control 
  src/main.cpp
)

add_dependencies(safe_remote_control
  hri_safety_sense_generate_messages_cpp
)

target_link_libraries(safe_remote_control 
  hri_safety_sense
  ${catkin_LIBRARIES}
)

install(TARGETS hri_safety_sense safe_remote_control
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY config launch scripts udev
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)