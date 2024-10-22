cmake_minimum_required(VERSION 3.0.2)
project(double_follow)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  tf2_ros
)

## Generate messages if you have custom message files
# add_message_files(
#   FILES
#   MyMessage.msg
# )

## Generate added messages
# generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs sensor_msgs tf tf2_ros
  # DEPENDS system_lib
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Build your executables here
# add_executable(my_node src/my_node.cpp)
# target_link_libraries(my_node ${catkin_LIBRARIES})
# add_dependencies(my_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Install targets
# install(TARGETS my_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Install other files
# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )
# install(DIRECTORY config/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
# )
# install(DIRECTORY scripts/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
# )
