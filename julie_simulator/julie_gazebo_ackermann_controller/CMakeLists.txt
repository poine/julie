cmake_minimum_required(VERSION 2.8.3)
project(julie_gazebo_ackermann_controller)


find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  geometry_msgs
  nav_msgs
  robotnik_msgs
  roscpp
  sensor_msgs
  tf
  diagnostic_msgs
  diagnostic_updater
)

catkin_package(
 CATKIN_DEPENDS robotnik_msgs ackermann_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(julie_gazebo_ackermann_controller_node src/julie_gazebo_ackermann_controller.cpp)

set(CMAKE_CXX_FLAGS "-Wall -std=c++11 ${CMAKE_CXX_FLAGS}")

add_dependencies(julie_gazebo_ackermann_controller_node julie_gazebo_ackermann_controller_node_generate_messages_cpp)
add_dependencies(julie_gazebo_ackermann_controller_node robotnik_msgs_generate_messages_cpp)


target_link_libraries(julie_gazebo_ackermann_controller_node
   ${catkin_LIBRARIES}
)
