#ifndef JULIE_CONTROL__JULIE_HARDWARE_INTERFACE_H
#define JULIE_CONTROL__JULIE_HARDWARE_INTERFACE_H

// example: https://github.com/ROBOTIS-OP/robotis_op_ros_control/blob/master/include/robotis_op_ros_control/robotis_op_hardware_interface.h

#include <ros/ros.h>
// ros_control
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>

#define NB_JOINTS 3 // two wheels plus steering

class JulieHardwareInterface : public hardware_interface::RobotHW
{

 public:
  JulieHardwareInterface();
  bool start();
  void read();
  void write();
  bool shutdown();
   
 private:
  hardware_interface::JointStateInterface    js_interface_;
  hardware_interface::EffortJointInterface   ej_interface_;
  hardware_interface::PositionJointInterface pj_interface_;
 
  // Joints
  double joint_position_[NB_JOINTS];
  double joint_velocity_[NB_JOINTS];
  double joint_effort_[NB_JOINTS];
  double joint_effort_command_[NB_JOINTS];
  double joint_position_command_[NB_JOINTS];
};
  
#endif // JULIE_CONTROL__JULIE_HARDWARE_INTERFACE_H
