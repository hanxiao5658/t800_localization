cmake_minimum_required(VERSION 3.0.2)
project(t800_localization)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
  pcl_ros
  pcl_conversions
  fast_gicp
)
find_package(PCL REQUIRED COMPONENTS common io filters registration kdtree)

catkin_package(CATKIN_DEPENDS roscpp rospy nav_msgs geometry_msgs sensor_msgs
               std_msgs std_srvs tf2_ros pcl_ros pcl_conversions fast_gicp)
include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(fastgicp_relocalizer src/fastgicp_relocalizer.cpp)
add_dependencies(fastgicp_relocalizer ${catkin_EXPORTED_TARGETS})
target_link_libraries(fastgicp_relocalizer ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# 保留之前建立的参考位姿采集工具。
catkin_install_python(PROGRAMS
  scripts/capture_reference_pose.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS fastgicp_relocalizer
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".gitkeep" EXCLUDE)
