cmake_minimum_required(VERSION 2.8.3)
project(julie_control)

############
## Config ##
############
add_compile_options(-std=c++11)
catkin_python_setup()
find_package(catkin REQUIRED COMPONENTS  roscpp
                                         rospy
                                         controller_interface
                                         controller_manager)
catkin_package(
  INCLUDE_DIRS include)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_library(julie_ackermann_controller
  src/julie_ackermann_controller.cpp)
  
target_link_libraries(julie_ackermann_controller
  ${catkin_LIBRARIES}
  )
#############
## Install ##
#############
install(FILES
  julie_ackerman_controller_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


# Install python programs
catkin_install_python(PROGRAMS scripts/path_editor
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



		    
