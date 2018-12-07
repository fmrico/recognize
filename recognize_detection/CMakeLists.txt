cmake_minimum_required(VERSION 2.8.3)
project(recognize_detection)

add_compile_options(-std=c++11)
add_definitions(-DOPENCV)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  sensor_msgs
  recognize_core_libs
  recognize_msgs
  roslint
  roslib
  image_transport
  recognize_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES recognize_capture
#  CATKIN_DEPENDS cv_bridge roscpp sensor_msgs recognize_core_libs recognize_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(recognize_capture
  src/recognize_capture/CameraImageProvider.cpp
  src/recognize_capture/ROSImageProvider.cpp
)
target_link_libraries(recognize_capture
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
)

add_executable(detection_node
  src/detection_node.cpp
)

add_dependencies(detection_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(detection_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   recognize_capture
)

add_executable(image_publisher_node
  src/image_publisher_node.cpp
)

add_dependencies(image_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(image_publisher_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
)


install(TARGETS detection_node recognize_capture
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

roslint_cpp(
 src/detection_node.cpp
 src/image_publisher_node.cpp
 include/recognize_capture/ImageProvider.h
 src/recognize_capture/ROSImageProvider.cpp include/recognize_capture/ROSImageProvider.h
 src/recognize_capture/CameraImageProvider.cpp include/recognize_capture/CameraImageProvider.h
)

catkin_add_gtest(test_capture test/test_capture.cpp)
if(TARGET test_capture)
   target_link_libraries(test_capture ${OpenCV_LIBS} recognize_capture ${catkin_LIBRARIES} gtest)
endif()
