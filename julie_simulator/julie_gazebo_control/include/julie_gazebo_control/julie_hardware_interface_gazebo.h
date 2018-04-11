#ifndef JULIE_GAZEBO_CONTROL__JULIE_HARDWARE_INTERFACE_GAZEBO_H
#define JULIE_GAZEBO_CONTROL__JULIE_HARDWARE_INTERFACE_GAZEBO_H

#include <ros/ros.h>
#include <gazebo_ros_control/robot_hw_sim.h>

namespace julie_hardware_gazebo {
  class JulieHardwareInterface : public gazebo_ros_control::RobotHWSim
  {
  public:
    JulieHardwareInterface();
    virtual bool initSim(
		 const std::string& robot_namespace,
		 ros::NodeHandle model_nh,
		 gazebo::physics::ModelPtr parent_model,
		 const urdf::Model *const urdf_model,
		 std::vector<transmission_interface::TransmissionInfo> transmissions);
    virtual void readSim(ros::Time time, ros::Duration period);
    virtual void writeSim(ros::Time time, ros::Duration period);

  private:
    
  };

  
}


#endif // JULIE_GAZEBO_CONTROL__JULIE_HARDWARE_INTERFACE_GAZEBO_H
