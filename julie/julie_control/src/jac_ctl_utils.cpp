#include "julie_control/jac_ctl_utils.h"

namespace julie_controller {

  JACVelRef::JACVelRef(double omega, double xi):
    omega_(omega),
    xi_(xi),
    v_(0.),
    a_(0.) {
    
  }

  void JACVelRef::update(const double& setpoint, const ros::Duration& dt) {
    const double tau = 0.75;
    a_ = -1/tau*(v_-setpoint);
    v_ += a_*dt.toSec();
  }



  JACDebugPublisher::JACDebugPublisher():
    publish_period_(1.0 / 50.) {}

  void JACDebugPublisher::init(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
    _pub_.reset(new realtime_tools::RealtimePublisher<julie_control::JAC_Debug>(controller_nh, "ctl_debug", 100));
  }

  void JACDebugPublisher::starting(const ros::Time& now) {
    last_state_publish_time_ = now;
   }
  
  void JACDebugPublisher::publish(const double vsp, const double vref, const double vmeas, const ros::Time& now) {
    if (last_state_publish_time_ + publish_period_ < now) {
      last_state_publish_time_ += publish_period_;
      if (_pub_->trylock()) {
	_pub_->msg_.vel_sp = vsp;
	_pub_->msg_.vel_ref = vref;
	_pub_->msg_. vel_meas = vmeas;
	_pub_->unlockAndPublish();
      }
    }
  }


}

