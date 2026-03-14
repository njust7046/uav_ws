cmake_minimum_required(VERSION 3.0.2)
project(lidar_to_mavros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp nav_msgs geometry_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(lidar_to_mavros src/lidar_to_mavros.cpp)
target_link_libraries(lidar_to_mavros ${catkin_LIBRARIES})
