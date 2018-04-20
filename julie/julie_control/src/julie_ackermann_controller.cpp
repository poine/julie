#include <julie_control/julie_ackermann_controller.h>



namespace julie_controller {
  JulieAckermannController::JulieAckermannController():
    steering_sp_(0.),
    speed_sp_(0.) {
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
    hardware_interface::EffortJointInterface* e2 = hw->get<hardware_interface::EffortJointInterface>();
    left_axle_joint_  = e2->getHandle("left_rear_axle_joint");
    right_axle_joint_ = e2->getHandle("right_rear_axle_joint");

    //jod_.init();
    
    //sub_command_ = controller_nh.subscribe("cmd_vel", 1, &JulieAckermannController::cmdVelCallback, this);
    sub_command_ = controller_nh.subscribe("cmd_ack", 1, &JulieAckermannController::cmdVelCallback, this);
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
    double _v = -0.5;//*sin(secs);
    left_axle_joint_.setCommand(_v);
    right_axle_joint_.setCommand(_v);

    //double _v2 = 0.5*sin(0.33*secs);
    double left_wheel_rvel_ = left_axle_joint_.getVelocity();
    double right_wheel_rvel_ = right_axle_joint_.getVelocity();
    //input_manager_.update(now);
    jod_.update(left_wheel_rvel_, right_wheel_rvel_, left_axle_joint_.getPosition(), now);

    //ROS_INFO("sp %f meas %f", steering_sp_, left_steering_joint_.getPosition());
    left_steering_joint_.setCommand(steering_sp_);
    right_steering_joint_.setCommand(steering_sp_);
  }

  /*******************************************************************************
   *
   *
   *******************************************************************************/
  void JulieAckermannController::stopping(const ros::Time&) {

  }

  /*******************************************************************************
   *
   *
   *******************************************************************************/
  void JulieAckermannController::cmdVelCallback(const ackermann_msgs::AckermannDriveStamped &msg) {
    steering_sp_ = msg.drive.steering_angle;
    speed_sp_ = msg.drive.speed;
    //ROS_INFO("JulieAckermannController::cmdVelCallback() %f", steering_sp_ );
    
  }
  PLUGINLIB_EXPORT_CLASS(julie_controller::JulieAckermannController, controller_interface::ControllerBase);
  
}

