#ifndef JULIE_CONTROL__JAC_ODOMETRY_H
#define JULIE_CONTROL__JAC_ODOMETRY_H

#include <ros/ros.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/function.hpp>

namespace julie_controller {

  namespace bacc = boost::accumulators;

  class JulieOdometry {

  public:
    JulieOdometry(size_t velocity_rolling_window_size = 10);
    void init(double wheel_base, size_t velocity_rolling_window_size);
    void starting(const ros::Time& now);
    void update(const double left_wheel_joint_velocity, 
		const double right_wheel_joint_velocity,
		const double steering_joint_position,
		const ros::Time &now);
    void setWheelbase(double wheelbase) { wheelbase_ = wheelbase; }
    void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);
    double getLinear() const { return linear_; }
    
  private:
    /// Rolling mean accumulator and window:
    typedef bacc::accumulator_set<double, bacc::stats<bacc::tag::rolling_mean> > RollingMeanAcc;
    typedef bacc::tag::rolling_window RollingWindow;

    void resetAccumulators();

    /// Current timestamp:
    ros::Time timestamp_;
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
    /// Rolling mean accumulators for the linar and angular velocities:
    size_t velocity_rolling_window_size_;
    RollingMeanAcc linear_acc_;
    RollingMeanAcc angular_acc_;
  };
  
}


#endif // JULIE_CONTROL__JAC_ODOMETRY
