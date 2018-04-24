cmake_minimum_required(VERSION 2.8.3)
project(julie_control)

############
## Config ##
############
#set(CMAKE_VERBOSE_MAKEFILE ON)
add_compile_options(-std=c++11)
catkin_python_setup()
find_package(catkin REQUIRED COMPONENTS  roscpp
                                         rospy
                                         controller_interface
                                         controller_manager
					 std_msgs
					 message_generation)
add_message_files(
  FILES
  RawOdom.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES julie_ackermann_controller
  CATKIN_DEPENDS message_runtime)


###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_library(julie_ackermann_controller
  src/julie_ackermann_controller.cpp
  src/jac_input_manager.cpp
  src/jac_odometry.cpp
  src/jac_publisher.cpp
  src/jac_raw_odom_publisher.cpp
  )
target_link_libraries(julie_ackermann_controller
  ${catkin_LIBRARIES}
  )
add_executable(julie_hardware_interface_node
  src/julie_hardware_interface.cpp
)
target_link_libraries(julie_hardware_interface_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############
install(FILES
  julie_ackermann_controller_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(TARGETS julie_ackermann_controller
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install python programs
catkin_install_python(PROGRAMS scripts/path_editor
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



		    
