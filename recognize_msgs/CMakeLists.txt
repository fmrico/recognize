cmake_minimum_required(VERSION 2.8.3)
project(recognize_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  sensor_msgs
  std_msgs
  message_generation
)

add_message_files(
   FILES
   ArrayBoundingBox.msg
   BoundingBox.msg
   ImageLabeler.msg
 )

generate_messages(
   DEPENDENCIES
   sensor_msgs
   std_msgs
)

catkin_package(
   CATKIN_DEPENDS message_runtime sensor_msgs std_msgs
)

install(DIRECTORY include/recognize_msgs/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN "*.h")
