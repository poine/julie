#include "julie_control/jac_ctl_utils.h"

namespace julie_controller {

  JACVelRef::JACVelRef(double omega, double xi):
    omega_(omega),
    xi_(xi) {
    
  }

  void JACVelRef::update(const double& setpoint, const ros::Duration&) {
    
    
  }
}
