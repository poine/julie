#include <julie_gazebo_control/julie_hardware_interface_gazebo.h>

#define __NAME "julie_hardware_interface_gazebo"

namespace julie_hardware_gazebo {
  JulieHardwareInterface::JulieHardwareInterface() {
    ROS_INFO_STREAM_NAMED( __NAME, "JulieHardwareInterface::JulieHardwareInterface");

  }

  bool JulieHardwareInterface::initSim(
				       const std::string& robot_namespace,
				       ros::NodeHandle model_nh,
				       gazebo::physics::ModelPtr parent_model,
				       const urdf::Model *const urdf_model,
				       std::vector<transmission_interface::TransmissionInfo> transmissions) {
    
    return true;
  }

  void JulieHardwareInterface::readSim(ros::Time time, ros::Duration period) {
  }
  void JulieHardwareInterface::writeSim(ros::Time time, ros::Duration period) {
  }
}
  
  
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(julie_hardware_gazebo::JulieHardwareInterface, gazebo_ros_control::RobotHWSim)
