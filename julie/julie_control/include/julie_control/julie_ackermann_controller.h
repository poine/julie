#ifndef JULIE_CONTROL__JULIE_ACKERMANN_CONTROLLER_H
#define JULIE_CONTROL__JULIE_ACKERMANN_CONTROLLER_H
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

#include "julie_control/jac_input_manager.h"

namespace julie_controller {
  
  class JulieAckermannController:
    public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface, hardware_interface::PositionJointInterface>
    {
    public:
      JulieAckermannController();
      bool init(hardware_interface::RobotHW* hw,
		ros::NodeHandle& root_nh,
		ros::NodeHandle& controller_nh);
      void starting(const ros::Time& now);
      void update(const ros::Time& , const ros::Duration&);
      void stopping(const ros::Time&);
	
    private:
      // values output to the hardware interface
      double steering_angle_;
      hardware_interface::JointHandle left_steering_joint_;
      hardware_interface::JointHandle right_steering_joint_;
      hardware_interface::JointHandle left_axle_joint_;
      hardware_interface::JointHandle right_axle_joint_;
    };
  
}


#endif // JULIE_CONTROL__JULIE_ACKERMANN_CONTROLLER_H
