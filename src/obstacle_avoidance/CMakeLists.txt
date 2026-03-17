cmake_minimum_required(VERSION 2.8.3)
project(obstacle_avoidance)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  mavros_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf
  pcl_ros
  pcl_conversions
)

find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs mavros_msgs nav_msgs sensor_msgs std_msgs tf pcl_ros pcl_conversions
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(avoidance_node src/avoidance_node.cpp)
target_link_libraries(avoidance_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
