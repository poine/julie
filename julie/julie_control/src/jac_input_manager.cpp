#include <julie_control/jac_input_manager.h>

namespace julie_controller {

  /*******************************************************************************
   *
   *
   *******************************************************************************/
  JulieInputManager::JulieInputManager() {

  }

  /*******************************************************************************
   *
   *
   *******************************************************************************/
  bool JulieInputManager::init(hardware_interface::RobotHW* hw, ros::NodeHandle &controller_nh) {

    sub_command_ = controller_nh.subscribe("cmd_ack", 1, &JulieInputManager::cmdVelCallback, this);
    return true;
  }

  /*******************************************************************************
   *
   *
   *******************************************************************************/

  void JulieInputManager::update(const ros::Time& now) {

    rt_commands_ = *(command_.readFromRT());
    const double dt = (now - rt_commands_.stamp).toSec();
    if (dt > 0.5) {
      rt_commands_.steering = 0.;
      rt_commands_.speed = 0.;
    }
    
  }

  /*******************************************************************************
   *
   *
   *******************************************************************************/
  void JulieInputManager::cmdVelCallback(const ackermann_msgs::AckermannDriveStamped &msg) {
    nrt_ros_command_struct_.steering   = msg.drive.steering_angle;
    nrt_ros_command_struct_.speed   = msg.drive.speed;
    nrt_ros_command_struct_.stamp = ros::Time::now();
    command_.writeFromNonRT (nrt_ros_command_struct_);
  }
  
}
