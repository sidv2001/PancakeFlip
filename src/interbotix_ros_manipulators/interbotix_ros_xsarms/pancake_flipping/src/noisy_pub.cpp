#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "interbotix_xs_msgs/JointGroupCommand.h"
#include "interbotix_xs_msgs/JointSingleCommand.h"
#include "interbotix_xs_msgs/RobotInfo.h"

interbotix_xs_msgs::JointGroupCommand joint_group_cmds;         // globally available joint_group_commands message
interbotix_xs_msgs::JointSingleCommand joint_single_cmds;         // globally available joint_states message

/// @brief Joint state callback function
/// @param msg - updated joint states
void joint_group_cmd_cb(const interbotix_xs_msgs::JointGroupCommand &msg)
{
  joint_group_cmds = msg;
}
void joint_single_cmd_cb(const interbotix_xs_msgs::JointSingleCommand &msg)
{
  joint_single_cmds = msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "xsarm_puppet_single");
  ros::NodeHandle n;

  // Subscribe to the robot's joint states and publish those states as joint commands so that rosbag can record them
  ros::Subscriber sub_group_cmds = n.subscribe("clean_cmds/joint_group", 1, joint_group_cmd_cb);
  ros::Subscriber sub_single_cmds = n.subscribe("clean_cmds/joint_single", 1, joint_single_cmd_cb);
  ros::Publisher pub_group = n.advertise<interbotix_xs_msgs::JointGroupCommand>("commands/joint_group", 1);
  ros::Publisher pub_single = n.advertise<interbotix_xs_msgs::JointSingleCommand>("commands/joint_single", 1);
  // Get some robot info to figure out how many joints the robot has
  ros::ServiceClient srv_robot_info = n.serviceClient<interbotix_xs_msgs::RobotInfo>("get_robot_info");

  ros::Rate loop_rate(50);
  bool success;

  // Wait for the 'arm_node' to finish initializing
  while ((pub_group.getNumSubscribers() < 1 || joint_group_cmds.cmd.size() < 1) && ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  // Get the number of joints that the first robot has
  interbotix_xs_msgs::RobotInfo arm_info_srv;
  arm_info_srv.request.cmd_type = "group";
  arm_info_srv.request.name = "arm";
  success = srv_robot_info.call(arm_info_srv);
  if (!success)
  {
    ROS_ERROR("Could not get info on the 'arm' group.");
    return 1;
  }

  // Get gripper info from the first robot
  interbotix_xs_msgs::RobotInfo gripper_info_srv;
  gripper_info_srv.request.cmd_type = "single";
  gripper_info_srv.request.name = "gripper";
  success = srv_robot_info.call(gripper_info_srv);
  if (!success)
  {
    ROS_ERROR("Could not get info on the 'gripper' joint.");
    return 1;
  }

  size_t cntr = 0;
  while (ros::ok())
  {
    pub_group.publish(joint_group_cmds);
    pub_single.publish(joint_single_cmds);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
