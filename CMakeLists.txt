cmake_minimum_required(VERSION 2.8.3)
project(canyonero_ros_controller)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  rosconsole
  roscpp
  roslib
  sensor_msgs
  std_msgs
)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES canyonero_ros_controller
  CATKIN_DEPENDS cv_bridge image_transport rosconsole roscpp roslib sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(realRosClass SHARED src/realRosClass.cpp)

## C++ executable: Canyonero_ros_controller_node
add_executable(canyonero_ros_controller_node src/ros_controller.cpp)
target_link_libraries(canyonero_ros_controller_node realRosClass ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${ROS_LIBRARIES} ncurses)

# Install targets
#install(TARGETS canyonero_ros_controller_node
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

message("CMake is done!")
message("Exit compilation")