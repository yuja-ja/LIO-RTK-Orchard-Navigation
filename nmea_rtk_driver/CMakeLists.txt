cmake_minimum_required(VERSION 3.0.2)
project(nmea_rtk_driver)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  geometry_msgs
  message_generation
  roscpp
  sensor_msgs
  serial
  std_msgs
)

add_message_files(
  FILES
  ReceiverStatus.msg
  RtkEpoch.msg
  RtkHeading.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES nmea_protocol
  CATKIN_DEPENDS
    diagnostic_msgs
    geometry_msgs
    message_runtime
    roscpp
    sensor_msgs
    serial
    std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(nmea_protocol
  src/epoch_assembler.cpp
  src/nmea_parser.cpp
)
target_link_libraries(nmea_protocol ${catkin_LIBRARIES})

add_executable(nmea_rtk_driver_node
  src/nmea_rtk_driver_node.cpp
)
add_dependencies(nmea_rtk_driver_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(nmea_rtk_driver_node
  nmea_protocol
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  add_executable(nmea_protocol_test
    test/nmea_protocol_test.cpp
  )
  target_link_libraries(nmea_protocol_test nmea_protocol)
  add_test(NAME nmea_protocol_test COMMAND nmea_protocol_test)
endif()

install(TARGETS nmea_protocol nmea_rtk_driver_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
