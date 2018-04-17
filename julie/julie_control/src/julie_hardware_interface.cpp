#include "julie_control/julie_hardware_interface.h"

#include <controller_manager/controller_manager.h>


#define __NAME "julie_hardware_interface"
const std::string joint_name_[NB_JOINTS] = {"left_rear_axle","right_rear_axle", "left_steering_joint"};
#define SAMPLE_RATE_HZ 100
#define DT (1./SAMPLE_RATE_HZ)

JulieHardwareInterface::JulieHardwareInterface() {
  // https://github.com/ros-controls/ros_control/wiki/hardware_interface
  // register joints
  for (int i=0; i<NB_JOINTS; i++) {
    joint_position_[i] = 0.;
    joint_velocity_[i] = 0.;
    joint_effort_[i] = 0.;
    joint_effort_command_[i] = 0.;
    joint_position_command_[i] = 0.;
    // register join state interface
    hardware_interface::JointStateHandle _state_handle(joint_name_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
    js_interface_.registerHandle(_state_handle);
    // register joint position or vel interface
    if (i<2) {
      hardware_interface::JointHandle _handle_vel(js_interface_.getHandle(joint_name_[i]), &joint_velocity_command_[i]);
      vj_interface_.registerHandle(_handle_vel);
    }
    else {
      hardware_interface::JointHandle _handle_pos(js_interface_.getHandle(joint_name_[i]), &joint_position_command_[i]);
      vj_interface_.registerHandle(_handle_pos);
    }
  }

 
  
  registerInterface(&js_interface_);
  registerInterface(&ej_interface_);
  registerInterface(&vj_interface_);
  registerInterface(&pj_interface_);


}

bool JulieHardwareInterface::start(){
 return true;
}

void JulieHardwareInterface::read(){
}

void JulieHardwareInterface::write(){
}

bool JulieHardwareInterface::shutdown(){
 return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, __NAME);
  ROS_INFO_STREAM_NAMED(__NAME, "Julie Hardware node starting...");
  ros::AsyncSpinner spinner(1);
  spinner.start();
 
  JulieHardwareInterface hw;
  if (!hw.start()) {
    ROS_ERROR_STREAM_NAMED(__NAME, "Julie Hardware failed to initialize. bailling out...");
    return -1;
  }

  ros::NodeHandle nh;
  controller_manager::ControllerManager cm(&hw, nh);
  ros::Duration period(DT);
  while (ros::ok())
  {
    hw.read();
    cm.update(ros::Time::now(), period);
    hw.write();
    period.sleep();
  }
  ROS_INFO_STREAM_NAMED(__NAME, "Julie Hardware node exiting...");
  if (!hw.shutdown()) {
    ROS_ERROR_STREAM_NAMED(__NAME, "Julie Hardware failed to shutdown. bailling out...");
    return -1;
  }
  return 0;
}
