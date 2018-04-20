#include <julie_control/julie_ackermann_controller.h>



namespace julie_controller {
  JulieAckermannController::JulieAckermannController():
    steering_angle_(0.) {
    ROS_INFO("JulieAckermannController::JulieAckermannController()");
  }

  /*******************************************************************************
   *
   *
   *******************************************************************************/
  bool JulieAckermannController::init(hardware_interface::RobotHW* hw,
				      ros::NodeHandle& root_nh,
				      ros::NodeHandle& controller_nh) {
    ROS_INFO("JulieAckermannController::init()");
    hardware_interface::PositionJointInterface* e = hw->get<hardware_interface::PositionJointInterface>();
    //left_steering_joint_ = e->getHandle("left_steering_joint");
    //right_steering_joint_ = e->getHandle("right_steering_joint");
    hardware_interface::EffortJointInterface* e2 = hw->get<hardware_interface::EffortJointInterface>();
    left_axle_joint_  = e2->getHandle("left_rear_axle_joint");
    right_axle_joint_ = e2->getHandle("right_rear_axle_joint");
    return true;
  }

  /*******************************************************************************
   *
   *
   *******************************************************************************/
  void JulieAckermannController::starting(const ros::Time& now) {
    ROS_INFO("JulieAckermannController::starting()");
  }
  
  /*******************************************************************************
   *
   *
   *******************************************************************************/
  void JulieAckermannController::update(const ros::Time& now, const ros::Duration& dt) {
    //ROS_INFO("JulieAckermannController::update()");
    double secs = now.toSec();
    double _v = 0;//0.2*sin(secs);
    //left_steering_joint_.setCommand(_v);
    //right_steering_joint_.setCommand(_v);

    double _v2 = 0.1*sin(0.33*secs);
    left_axle_joint_.setCommand(_v2);
    right_axle_joint_.setCommand(_v2);
  }

  /*******************************************************************************
   *
   *
   *******************************************************************************/
  void JulieAckermannController::stopping(const ros::Time&) {

  }
  
  PLUGINLIB_EXPORT_CLASS(julie_controller::JulieAckermannController, controller_interface::ControllerBase);
  
}

