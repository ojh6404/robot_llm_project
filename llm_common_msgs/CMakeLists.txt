cmake_minimum_required(VERSION 3.0.2)
project(llm_common_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  jsk_recognition_msgs
  message_generation
)

add_message_files(
  FILES
  TaskSequence.msg
  ClipResultStamped.msg
)

add_service_files(
    FILES
    Query.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  jsk_recognition_msgs
)

catkin_package(
  CATKIN_DEPENDS
  std_msgs
  sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
