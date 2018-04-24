#ifndef JULIE_CONTROL__JAC_RAW_ODOM_PUBLISHER_H
#define JULIE_CONTROL__JAC_RAW_ODOM_PUBLISHER_H

#include <ros/ros.h>
#include <julie_control/RawOdom.h>


#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#define MAX_SENSOR_LEN 15
#define MIN_SENSOR_FOR_PUBLISH 10

namespace julie_controller {
  
  class JACRawOdomPublisher {
  public:
    JACRawOdomPublisher();
    void init(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
    //void starting(const ros::Time& now);
    void publish(const double left_wheel_angle, const double right_wheel_angle, const double steering_angle, const ros::Time& now);
  private:
    boost::shared_ptr<realtime_tools::RealtimePublisher<julie_control::RawOdom> > _pub_;
    int nb_data_;
    ros::Time stamp_[MAX_SENSOR_LEN];
    double left_wheel_angle_[MAX_SENSOR_LEN];
    double right_wheel_angle_[MAX_SENSOR_LEN];
    double steering_angle_[MAX_SENSOR_LEN];
  };
}
#endif // JULIE_CONTROL__JAC_RAW_ODOM_PUBLISHER_H
