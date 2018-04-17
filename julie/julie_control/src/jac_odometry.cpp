#include "julie_control/jac_odometry.h"

#include <ros/ros.h>

namespace julie_controller {
  
  JulieOdometry::JulieOdometry():
    steering_angle_(0.) {

    ROS_INFO("JulieOdometry::JulieOdometry()");

  }


  void JulieOdometry::update() {
    ROS_INFO("JulieOdometry::update()");
  }

  
}
