#ifndef JULIE_CONTROL__JULIE_ACKERMANN_CONTROLLER_H
#define JULIE_CONTROL__JULIE_ACKERMANN_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

#include "julie_control/jac_input_manager.h"
#include "julie_control/jac_odometry.h"
#include "julie_control/jac_publisher.h"
#include "julie_control/jac_raw_odom_publisher.h"

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
      JulieInputManager input_manager_;
      JulieOdometry odometry_;
      JACPublisher publisher_;
      JACRawOdomPublisher raw_odom_publisher_;
      hardware_interface::JointHandle left_steering_joint_;
      hardware_interface::JointHandle right_steering_joint_;
      hardware_interface::JointHandle left_axle_joint_;
      hardware_interface::JointHandle right_axle_joint_;
    };
  
}


#endif // JULIE_CONTROL__JULIE_ACKERMANN_CONTROLLER_H
