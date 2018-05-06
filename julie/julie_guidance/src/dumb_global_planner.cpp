#include <fstream>
#include <pluginlib/class_list_macros.h>
#include "julie_guidance/dumb_global_planner.h"

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::DumbGlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

std::istream& operator>>(std::istream& is, CoordinatePair& coordinates)
{
  is >> std::ws >> coordinates.x >> std::ws >> coordinates.y >> std::ws >> coordinates.theta >> std::ws;
  return is;
}

//Default Constructor
namespace global_planner {

  DumbGlobalPlanner::DumbGlobalPlanner (){
    ROS_INFO("in DumbGlobalPlanner::DumbGlobalPlanner");
  }

  DumbGlobalPlanner::DumbGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    ROS_INFO("in DumbGlobalPlanner::DumbGlobalPlanner2");
    initialize(name, costmap_ros);
  }


  void DumbGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    ROS_INFO("in DumbGlobalPlanner::initialize %s", name.c_str());

    ros::NodeHandle private_nh("~/" + name);
    std::string param_name = "/"+name+"/static_plan_path";
    ROS_INFO("in DumbGlobalPlanner::initialize %s", param_name.c_str());
    private_nh.param<std::string>(param_name, _static_plan_path, "/tmp/foo.path");

    
    ROS_INFO("  DumbGlobalPlanner readingin static plan from %s", _static_plan_path.c_str());
    std::ifstream infile(_static_plan_path);
    if (infile) {
      std::copy(std::istream_iterator<CoordinatePair>(infile), 
                std::istream_iterator<CoordinatePair>(),
                std::back_inserter(points));
    }
    else {
      ROS_ERROR("Couldn't open %s for reading", _static_plan_path.c_str());
    }
  }

  bool DumbGlobalPlanner::makePlan( const geometry_msgs::PoseStamped& start,
				    const geometry_msgs::PoseStamped& goal,
				    std::vector<geometry_msgs::PoseStamped>& plan ) {

    ROS_INFO("in DumbGlobalPlanner::makePlan : start %f %f", start.pose.position.x, start.pose.position.y);

#if 0
    double dx = goal.pose.position.x - start.pose.position.x;
    double dy = goal.pose.position.y - start.pose.position.y;
    double theta = atan2(dy, dx);
    int n_step = 20;
    //plan.push_back(start);
    ros::Time start_time = start.header.stamp;
    for (int i=0; i<n_step; i++){
      geometry_msgs::PoseStamped new_goal = goal;
      new_goal.header.stamp =  start_time + ros::Duration(i*0.1);
      tf::Quaternion goal_quat = tf::createQuaternionFromYaw(theta);
      ROS_INFO("yaw %f", theta);
      new_goal.pose.position.x = start.pose.position.x + dx*i/n_step;
      new_goal.pose.position.y = start.pose.position.y + dy*i/n_step;
      
      new_goal.pose.orientation.x = goal_quat.x();
      new_goal.pose.orientation.y = goal_quat.y();
      new_goal.pose.orientation.z = goal_quat.z();
      new_goal.pose.orientation.w = goal_quat.w();
      
      plan.push_back(new_goal);
    }
    
    //goal.header.stamp = start_time + ros::Duration(2.1);
    //plan.push_back(goal);
#else
    for(auto& it: points) {
      //ROS_INFO("x %f y %f yaw %f", it.x, it.y, it.theta);
      geometry_msgs::PoseStamped new_goal = goal;
      new_goal.pose.position.x = it.x;// + start.pose.position.x;
      new_goal.pose.position.y = it.y;// + start.pose.position.y;
      tf::Quaternion goal_quat = tf::createQuaternionFromYaw(it.theta);
      new_goal.pose.orientation.x = goal_quat.x();
      new_goal.pose.orientation.y = goal_quat.y();
      new_goal.pose.orientation.z = goal_quat.z();
      new_goal.pose.orientation.w = goal_quat.w();
      plan.push_back(new_goal);
    }

#endif
    return true;
  }


  


  
};
