#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "interbotix_xs_msgs/JointGroupCommand.h"
#include "interbotix_xs_msgs/JointSingleCommand.h"
#include "interbotix_xs_msgs/RobotInfo.h"
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarkersMultiCam.h>
#include <message_filters/synchronizer.h>
#include <boost/shared_ptr.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

ros::Publisher pub_ar_tags;
sensor_msgs::JointState joints;         // globally available joint_states message

void ar_pose_cb(const boost::shared_ptr<const ar_track_alvar_msgs::AlvarMarkers>& ar_cam1,
                const boost::shared_ptr<const ar_track_alvar_msgs::AlvarMarkers>& ar_cam2,
                const boost::shared_ptr<const ar_track_alvar_msgs::AlvarMarkers>& ar_cam3,
                const boost::shared_ptr<const sensor_msgs::JointState>& joint_states) {
  ROS_WARN("\n\n\n CP0 \n\n\n");
  joints = *joint_states;
  ar_track_alvar_msgs::AlvarMarkersMultiCam ar_cams;
  ar_cams.cam1_marker = ar_cam1->markers[0];
  ar_cams.cam2_marker = ar_cam2->markers[0];
  ar_cams.cam3_marker = ar_cam3->markers[0];
  ar_cams.joint_states = *joint_states;
  ROS_WARN("\n\n\n CP1 \n\n\n");

  /////// Create average of channels ////////

  // Create arrays of valid data
  std::vector <ar_track_alvar_msgs::AlvarMarker> valid_markers;
  if (ar_cam1->markers[0].header.frame_id != "null") valid_markers.push_back(ar_cam1->markers[0]);
  if (ar_cam2->markers[0].header.frame_id != "null") valid_markers.push_back(ar_cam2->markers[0]);
  if (ar_cam3->markers[0].header.frame_id != "null") valid_markers.push_back(ar_cam3->markers[0]);
  ROS_WARN("\n\n\n CP2 \n\n\n");

  // Loop through valid markers
  int total_valid = valid_markers.size();
  if (total_valid == 0) {
    ar_cams.avg_marker = ar_cam1->markers[0]; // all markers are equivalently dummy markers 
    pub_ar_tags.publish(ar_cams);
    return;
  }
  ROS_WARN("\n\n\n CP3 \n\n\n");

  long long values [7];           // [px, py, pz, qx, qy, qz, qw]
  for (ar_track_alvar_msgs::AlvarMarker const& valid_marker : valid_markers) {
    values[0] += valid_marker.pose.pose.position.x;
    values[1] += valid_marker.pose.pose.position.y;
    values[2] += valid_marker.pose.pose.position.z;
    values[3] += valid_marker.pose.pose.orientation.x;
    values[4] += valid_marker.pose.pose.orientation.y;
    values[5] += valid_marker.pose.pose.orientation.z;
    values[6] += valid_marker.pose.pose.orientation.w;
  }
  ROS_WARN("\n\n\n CP4\n\n\n");

  ar_cams.avg_marker.pose.pose.position.x    = values[0]/total_valid;
  ar_cams.avg_marker.pose.pose.position.y    = values[1]/total_valid;
  ar_cams.avg_marker.pose.pose.position.z    = values[2]/total_valid;
  ar_cams.avg_marker.pose.pose.orientation.x = values[3]/total_valid;
  ar_cams.avg_marker.pose.pose.orientation.y = values[4]/total_valid;
  ar_cams.avg_marker.pose.pose.orientation.z = values[5]/total_valid;
  ar_cams.avg_marker.pose.pose.orientation.w = values[6]/total_valid;
  ROS_WARN("\n\n\n CP5\n\n\n");

  pub_ar_tags.publish(ar_cams);
  ROS_WARN("\n\n\n CP6\n\n\n");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "xsarm_puppet_single");
  ros::NodeHandle n;

  typedef message_filters::sync_policies::ApproximateTime<ar_track_alvar_msgs::AlvarMarkers, ar_track_alvar_msgs::AlvarMarkers, ar_track_alvar_msgs::AlvarMarkers, sensor_msgs::JointState> MySyncPolicy;
  // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)

  // Subscribe to the robot's joint states and publish those states as joint commands so that rosbag can record them
  // ros::Subscriber sub_positions = n.subscribe("joint_states", 1, joint_state_cb);
  message_filters::Subscriber<sensor_msgs::JointState>           joint_pos_sub   (n, "joint_states",            1);
  message_filters::Subscriber<ar_track_alvar_msgs::AlvarMarkers> ar_pose_cam1_sub(n, "ar_pose_markers_camera1", 1);
  message_filters::Subscriber<ar_track_alvar_msgs::AlvarMarkers> ar_pose_cam2_sub(n, "ar_pose_markers_camera2", 1);
  message_filters::Subscriber<ar_track_alvar_msgs::AlvarMarkers> ar_pose_cam3_sub(n, "ar_pose_markers_camera3", 1);
  pub_ar_tags = n.advertise<ar_track_alvar_msgs::AlvarMarkersMultiCam>("ar_pose_markers_multi_cam", 1);
  ros::Publisher pub_group = n.advertise<interbotix_xs_msgs::JointGroupCommand>("clean_cmds/joint_group", 1);
  ros::Publisher pub_single = n.advertise<interbotix_xs_msgs::JointSingleCommand>("clean_cmds/joint_single", 1);
  // Get some robot info to figure out how many joints the robot has
  ros::ServiceClient srv_robot_info = n.serviceClient<interbotix_xs_msgs::RobotInfo>("get_robot_info");
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), ar_pose_cam1_sub, ar_pose_cam2_sub, ar_pose_cam3_sub, joint_pos_sub);
  ROS_WARN("\n\n\n BEFORE \n\n\n");
  sync.registerCallback(boost::bind(&ar_pose_cb, _1, _2, _3, _4));
  ROS_WARN("\n\n\n AFTER \n\n\n");

  ros::Rate loop_rate(60);
  bool success;
  ROS_WARN("\n\n\n CP1 \n\n\n");

  // Wait for the 'arm_node' to finish initializing
  while ((pub_group.getNumSubscribers() < 1) && ros::ok()) {
    ros::spinOnce();
    ROS_WARN_STREAM("\n\n\n\n");
    ROS_WARN_STREAM(std::to_string(pub_group.getNumSubscribers()));
    ROS_WARN_STREAM(joints.position.size() );
    ROS_WARN_STREAM(std::to_string(ros::ok()));
    loop_rate.sleep();
  }
  ROS_WARN("\n\n\n CP2 \n\n\n");

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
    // put joint positions from the robot as position commands for itself
    interbotix_xs_msgs::JointGroupCommand pos_msg;
    pos_msg.name = "arm";
    for (auto const& index : arm_info_srv.response.joint_state_indices)
      pos_msg.cmd.push_back(joints.position.at(index));
    pub_group.publish(pos_msg);

    interbotix_xs_msgs::JointSingleCommand single_msg;
    single_msg.name = "gripper";
    single_msg.cmd = joints.position.at(gripper_info_srv.response.joint_state_indices.at(0))*2;
    pub_single.publish(single_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
