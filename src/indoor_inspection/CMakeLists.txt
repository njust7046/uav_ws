cmake_minimum_required(VERSION 2.8.3)
project(indoor_inspection)

add_compile_options(-std=c++11)

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

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(inspection_node src/inspection_node.cpp)
target_link_libraries(inspection_node ${catkin_LIBRARIES})

# Install launch, config, worlds, scripts
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY worlds/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds)
install(PROGRAMS scripts/start_px4_sitl.sh scripts/setup_gazebo_env.sh
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)
