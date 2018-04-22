#include "julie_control/jac_odometry.h"

#include <ros/ros.h>

namespace julie_controller {
  
  JulieOdometry::JulieOdometry(size_t velocity_rolling_window_size):
    x_(0.0)
    , y_(0.0)
    , heading_(0.0)
    , linear_(0.0)
    , angular_(0.0)
    , wheelbase_(1.65)
    , wheel_radius_(0.47/2)
    , velocity_rolling_window_size_(velocity_rolling_window_size)
    , linear_acc_(RollingWindow::window_size = velocity_rolling_window_size)
    , angular_acc_(RollingWindow::window_size = velocity_rolling_window_size) {

    ROS_INFO("JulieOdometry::JulieOdometry()");

  }

  void JulieOdometry:: init(double wheel_base, size_t velocity_rolling_window_size) {
    setWheelbase(wheel_base);
    setVelocityRollingWindowSize(velocity_rolling_window_size);
  }
  
  void JulieOdometry::starting(const ros::Time& now) {
    // Reset accumulators and timestamp:
    resetAccumulators();
    timestamp_ = now;
  }

  void JulieOdometry::update(const double left_wheel_joint_velocity, 
			     const double right_wheel_joint_velocity,
			     const double steering_joint_position,
			     const ros::Time &now) {

    //ROS_INFO("JulieOdometry::update()");
    const double dt = (now - timestamp_).toSec();
    timestamp_ = now;

    const double linear = (left_wheel_joint_velocity + right_wheel_joint_velocity)/2.*wheel_radius_*dt;
    const double angular = linear * tan(steering_joint_position) / wheelbase_;
    /// integrate for position and heading
    const double curvature_radius = wheelbase_ / cos(M_PI/2.0 - steering_joint_position);
    if (fabs(curvature_radius) > 0.0001) {
      const double elapsed_distance = linear;
      const double elapsed_angle = elapsed_distance / curvature_radius;
      const double x_curvature = curvature_radius * sin(elapsed_angle);
      const double y_curvature = curvature_radius * (cos(elapsed_angle) - 1.0);
      const double wheel_heading = heading_ + steering_joint_position;
      y_ += x_curvature * sin(wheel_heading) + y_curvature * cos(wheel_heading);
      x_ += x_curvature * cos(wheel_heading) - y_curvature * sin(wheel_heading);
      heading_ += elapsed_angle;
    }
    /// Estimate speeds using a rolling mean to filter them out:
    linear_acc_(linear/dt);
    angular_acc_(angular/dt);

    linear_ = bacc::rolling_mean(linear_acc_);
    angular_ = bacc::rolling_mean(angular_acc_);

  }

  void JulieOdometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size) {
    velocity_rolling_window_size_ = velocity_rolling_window_size;
    resetAccumulators();
  }
  
  void JulieOdometry::resetAccumulators() {
    linear_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
    angular_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
  }
  
}
