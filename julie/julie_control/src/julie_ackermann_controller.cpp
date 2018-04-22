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

    const double wheel_base = 1.650;
    size_t velocity_rolling_window_size = 10;
    odometry_.init(wheel_base, velocity_rolling_window_size);
    //sub_command_ = controller_nh.subscribe("cmd_vel", 1, &JulieAckermannController::cmdVelCallback, this);
    sub_command_ = controller_nh.subscribe("cmd_ack", 1, &JulieAckermannController::cmdVelCallback, this);
    publisher_.init(root_nh, controller_nh);
    return true;
  }

  /*******************************************************************************
   *
   *
   *******************************************************************************/
  void JulieAckermannController::starting(const ros::Time& now) {
    ROS_INFO("JulieAckermannController::starting()");
    odometry_.starting(now);
    publisher_.starting(now);
  }
  
  /*******************************************************************************
   *
   *
   *******************************************************************************/
  void JulieAckermannController::update(const ros::Time& now, const ros::Duration& dt) {
    //ROS_INFO("JulieAckermannController::update()");
    //double secs = now.toSec();

    double left_wheel_rvel_ = left_axle_joint_.getVelocity();
    double right_wheel_rvel_ = right_axle_joint_.getVelocity();
    double steering_angle = (left_steering_joint_.getPosition()+right_steering_joint_.getPosition())/2.;
    //input_manager_.update(now);
    odometry_.update(left_wheel_rvel_, right_wheel_rvel_, steering_angle, now);

    double _vel_err = odometry_.getLinear() - speed_sp_;
    const double Kp = -0.4;
    double _vel_cmd = 0.1*speed_sp_ +  Kp*_vel_err;//*sin(secs);
    //ROS_DEBUG_THROTTLE(0.1, "vel %f", odometry_.getLinear());
    //ROS_INFO("vel %f sp %f", odometry_.getLinear(), speed_sp_);    
    left_axle_joint_.setCommand(_vel_cmd);
    right_axle_joint_.setCommand(_vel_cmd);

    //ROS_INFO("sp %f meas %f", steering_sp_, left_steering_joint_.getPosition());
    left_steering_joint_.setCommand(steering_sp_);
    right_steering_joint_.setCommand(steering_sp_);
    
    publisher_.publish(odometry_.getHeading(), odometry_.getX(), odometry_.getY(), odometry_.getLinear(), odometry_.getAngular(), now);
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

