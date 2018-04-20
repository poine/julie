#include "julie_control/jac_odometry.h"

#include <ros/ros.h>

namespace julie_controller {
  
  JulieOdometry::JulieOdometry():
      x_(0.0)
    , y_(0.0)
    , heading_(0.0)
    , linear_(0.0)
    , angular_(0.0)
    , wheelbase_(1.0)
    , wheel_radius_(0.03) {

    ROS_INFO("JulieOdometry::JulieOdometry()");

  }


  void JulieOdometry::update(const double left_wheel_joint_velocity, 
			     const double right_wheel_joint_velocity,
			     const double steering_joint_position,
			     const ros::Time &time) {

    //ROS_INFO("JulieOdometry::update()");

  }

  
}
