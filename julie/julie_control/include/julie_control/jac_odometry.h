#ifndef JULIE_CONTROL__JAC_ODOMETRY_H
#define JULIE_CONTROL__JAC_ODOMETRY_H
#include <ros/ros.h>

namespace julie_controller {
  
  class JulieOdometry {
    public:
      JulieOdometry();
      void update(const double left_wheel_joint_velocity, 
		  const double right_wheel_joint_velocity,
		  const double steering_joint_position,
		  const ros::Time &time);
    private:
      /// Current pose:
      double x_;        //   [m]
      double y_;        //   [m]
      double heading_;  // [rad]

      /// Current velocity:
      double linear_;  //   [m/s]
      double angular_; // [rad/s]

      /// Geometry
      double wheelbase_;
      double wheel_radius_;
    };
  
}


#endif // JULIE_CONTROL__JAC_ODOMETRY
