#ifndef JULIE_CONTROL__JULIE_ACKERMANN_CONTROLLER_H
#define JULIE_CONTROL__JULIE_ACKERMANN_CONTROLLER_H
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

namespace julie_controller {
  
  class JulieAckermannController:
    public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface, hardware_interface::PositionJointInterface>
    {
    public:
      JulieAckermannController();
      void update(const ros::Time& , const ros::Duration&);
    private:
      // values output to the hardware interface
      double steering_angle_;
    };
  
}


#endif // JULIE_CONTROL__JULIE_ACKERMANN_CONTROLLER_H
