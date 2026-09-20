cmake_minimum_required(VERSION 3.0.2)
project(rtk_truth_tum)

find_package(catkin REQUIRED COMPONENTS
  nmea_rtk_driver
  rosbag
  rospy
)

catkin_package(
  CATKIN_DEPENDS
    nmea_rtk_driver
    rosbag
    rospy
)

catkin_install_python(PROGRAMS
  scripts/extract_rtk_truth_from_bag.py
  scripts/transform_imu_tum_to_antenna.py
  scripts/calibrate_lever_arm_heading.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
