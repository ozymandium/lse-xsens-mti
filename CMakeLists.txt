cmake_minimum_required(VERSION 2.8.3)
project(lse_xsens_mti)

find_package(catkin REQUIRED COMPONENTS cereal_port gps_common nav_msgs roscpp sensor_msgs tf)

catkin_package(
 INCLUDE_DIRS src
 LIBRARIES lse_xsens_mti
 CATKIN_DEPENDS cereal_port gps_common nav_msgs roscpp sensor_msgs tf
 DEPENDS system_lib
)

include_directories(
  src
  ${catkin_INCLUDE_DIRS}
)

add_executable(mti_node
  src/mti_node.cpp
  src/MTi/MTi.cpp
  src/MTi/MTMessage.cpp
)

target_link_libraries(mti_node
  cereal_port
  ${catkin_LIBRARIES}
)

