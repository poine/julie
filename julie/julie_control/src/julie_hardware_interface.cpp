#include "julie_control/julie_hardware_interface.h"

#include <controller_manager/controller_manager.h>


#define __NAME "julie_hardware_interface"
const std::string joint_name_[NB_JOINTS] = {"left_wheel_joint","right_wheel_joint", "steering_joint"};
#define SAMPLE_RATE_HZ 100
#define DT (1./SAMPLE_RATE_HZ)

JulieHardwareInterface::JulieHardwareInterface() {

  // register joints
  for (int i=0; i<NB_JOINTS; i++) {
    joint_position_[i] = 0.;
    joint_velocity_[i] = 0.;
    joint_effort_[i] = 0.;
    joint_effort_command_[i] = 0.;
    joint_position_command_[i] = 0.;
    if (i<2) {
      ej_interface_.registerHandle(hardware_interface::JointHandle(
	js_interface_.getHandle(joint_name_[i]), &joint_effort_command_[i]));
    }
    else {
      pj_interface_.registerHandle(hardware_interface::JointHandle(
        js_interface_.getHandle(joint_name_[i]), &joint_position_command_[i]));
    }
  }
  registerInterface(&js_interface_);
  registerInterface(&ej_interface_);
  registerInterface(&pj_interface_);


}

bool JulieHardwareInterface::start(){
 return true;
}

void JulieHardwareInterface::write(){
}

void JulieHardwareInterface::read(){
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
