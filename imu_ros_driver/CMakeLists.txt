cmake_minimum_required(VERSION 3.0.2)
project(imu_ros_driver)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  serial
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs serial
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  imu_ros_driver_node
  src/imu_driver_node.cpp
)

target_link_libraries(
  imu_ros_driver_node
  ${catkin_LIBRARIES}
)

install(
  TARGETS imu_ros_driver_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)