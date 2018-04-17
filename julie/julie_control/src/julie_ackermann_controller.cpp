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
    left_steering_joint_ = e->getHandle("left_steering_joint");
    right_steering_joint_ = e->getHandle("right_steering_joint");
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
    double _v = sin(secs);
    left_steering_joint_.setCommand(_v);
    right_steering_joint_.setCommand(_v);
  }

  /*******************************************************************************
   *
   *
   *******************************************************************************/
  void JulieAckermannController::stopping(const ros::Time&) {

  }
  
  PLUGINLIB_EXPORT_CLASS(julie_controller::JulieAckermannController, controller_interface::ControllerBase);
  
}

