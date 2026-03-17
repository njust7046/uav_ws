cmake_minimum_required(VERSION 2.8.3)
project(circle_crossing)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  mavros_msgs
  nav_msgs
  std_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs mavros_msgs nav_msgs std_msgs tf
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(circle_crossing_node src/circle_crossing_node.cpp)
target_link_libraries(circle_crossing_node ${catkin_LIBRARIES})

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
