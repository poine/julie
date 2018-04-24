#ifndef JULIE_CONTROL__JAC_INPUT_MANAGER_H
#define JULIE_CONTROL__JAC_INPUT_MANAGER_H

#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <controller_interface/multi_interface_controller.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

namespace julie_controller {
  
  class JulieInputManager {
  public:
    JulieInputManager();
    bool init(hardware_interface::RobotHW* hw, ros::NodeHandle &controller_nh);
    void update(const ros::Time&);

    struct Commands {
      double steering;
      double speed;
      ros::Time stamp;
      Commands() : steering(0.0), speed(0.0), stamp(0.0) {}
    };
    realtime_tools::RealtimeBuffer<Commands> command_;
    Commands nrt_ros_command_struct_;
    ros::Subscriber sub_command_;
    Commands rt_commands_;

  private:
    void cmdVelCallback(const ackermann_msgs::AckermannDriveStamped &msg);
    
  };
  

}

#endif // JULIE_CONTROL__JAC_INPUT_MANAGER_H
