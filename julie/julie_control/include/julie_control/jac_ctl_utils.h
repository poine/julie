#ifndef JULIE_CONTROL__JAC_CTL_UTILS_H
#define JULIE_CONTROL__JAC_CTL_UTILS_H
#include <ros/ros.h>

#include <julie_control/JAC_Debug.h>


#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

namespace julie_controller {
  
  class JACVelRef {
  public:
    JACVelRef(double omega=1., double xi=0.7);
    void update(const double& setpoint, const ros::Duration&);
    double v_;
    double a_;
  private:
    double omega_;
    double xi_;
  };



  class JACDebugPublisher {
  public:
    JACDebugPublisher();
    void init(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
    void starting(const ros::Time& now);
    void publish(const double vsp, const double vref, const double vmeas, const ros::Time& now);

  private:
    boost::shared_ptr<realtime_tools::RealtimePublisher<julie_control::JAC_Debug> > _pub_;
    ros::Duration publish_period_;
    ros::Time last_state_publish_time_;
  };




  
}
#endif // JULIE_CONTROL__JAC_CTL_UTILS_H
