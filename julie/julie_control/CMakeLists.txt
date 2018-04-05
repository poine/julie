cmake_minimum_required(VERSION 2.8.3)
project(julie_control)

find_package(catkin REQUIRED COMPONENTS  rospy)
catkin_package()

# Install python modules
catkin_python_setup()
# Install python programs
catkin_install_python(PROGRAMS scripts/path_editor
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
