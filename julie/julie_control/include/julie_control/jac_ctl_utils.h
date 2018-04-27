#ifndef JULIE_CONTROL__JAC_CTL_UTILS_H
#define JULIE_CONTROL__JAC_CTL_UTILS_H
#include <ros/ros.h>

namespace julie_controller {
  
  class JACVelRef {
  public:
    JACVelRef(double omega=1., double xi=0.7);
    void update(const double& setpoint, const ros::Duration&);
  private:
    double omega_;
    double xi_;
  };


  




  
}
#endif // JULIE_CONTROL__JAC_CTL_UTILS_H
