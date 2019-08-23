cmake_minimum_required(VERSION 2.8.3)
project(moby)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  PWMCommand.msg
  GPS.msg
  Yaw.msg
  ArduinoSensors.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS message_generation message_runtime rospy std_msgs sensor_msgs geometry_msgs visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)