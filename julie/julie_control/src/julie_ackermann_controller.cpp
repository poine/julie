#include <julie_control/julie_ackermann_controller.h>



namespace julie_controller {
  JulieAckermannController::JulieAckermannController():
    steering_angle_(0.) {
    
  }

  void JulieAckermannController::update(const ros::Time& now, const ros::Duration& dt) {

  }
    
  PLUGINLIB_EXPORT_CLASS(julie_controller::JulieAckermannController, controller_interface::ControllerBase);
  
}

