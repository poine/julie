#include <julie_control/julie_ackermann_controller.h>



namespace julie_controller {
  /*******************************************************************************
   *
   *
   *******************************************************************************/
  JulieAckermannController::JulieAckermannController() {
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
    input_manager_.init( hw, controller_nh);
    publisher_.init(root_nh, controller_nh);
    raw_odom_publisher_.init(root_nh, controller_nh);
    return true;
  }

  /*******************************************************************************
   *
   *
   *******************************************************************************/
  void JulieAckermannController::starting(const ros::Time& now) {
    ROS_INFO("JulieAckermannController::starting()");
    odometry_.starting(now);
    //input_manager_.starting(now);
    publisher_.starting(now);
  }
  
  /*******************************************************************************
   *
   *
   *******************************************************************************/
  void JulieAckermannController::update(const ros::Time& now, const ros::Duration& dt) {
    
    double left_wheel_rvel = left_axle_joint_.getVelocity();
    double right_wheel_rvel = right_axle_joint_.getVelocity();
    double left_wheel_angle = left_axle_joint_.getPosition();
    double right_wheel_angle = right_axle_joint_.getPosition();
    double steering_angle = (left_steering_joint_.getPosition()+right_steering_joint_.getPosition())/2.;
    odometry_.update(left_wheel_rvel, right_wheel_rvel, steering_angle, now);

    input_manager_.update(now);
    double speed_setpoint = input_manager_.rt_commands_.speed;
    double steering_setpoint = input_manager_.rt_commands_.steering;

    vel_ref_.update(speed_setpoint, dt);
    
    double _vel_err = odometry_.getLinear() - speed_setpoint;
    const double Kp = -0.6;
    double _eff_cmd = 0.1*speed_setpoint +  Kp*_vel_err;
    left_axle_joint_.setCommand(_eff_cmd);
    right_axle_joint_.setCommand(_eff_cmd);

    left_steering_joint_.setCommand(steering_setpoint);
    right_steering_joint_.setCommand(steering_setpoint);
    
    publisher_.publish(odometry_.getHeading(), odometry_.getX(), odometry_.getY(), odometry_.getLinear(), odometry_.getAngular(), now);
    raw_odom_publisher_.publish(left_wheel_angle, right_wheel_angle, steering_angle, now);
    
  }

  /*******************************************************************************
   *
   *
   *******************************************************************************/
  void JulieAckermannController::stopping(const ros::Time&) {

  }

 
  
  PLUGINLIB_EXPORT_CLASS(julie_controller::JulieAckermannController, controller_interface::ControllerBase);
  
}

