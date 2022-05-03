#!/usr/bin/env python  
import utils 
import bagpy
import pandas as pd 
import json
import os
# import rospy

PKG_DIR = '/home/robotics/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/pancake_flipping'
BAG_DIR = os.path.join(PKG_DIR, 'bag')
CSV_BASE_DIR = os.path.join(PKG_DIR, 'csv')

bag_name = 'multicam.bag'
JOINT_STATES = "/wx250s/joint_states"
POSES = "/wx250s/ar_pose_markers_multi_cam"
# CAM1 = "camera1"
# CAM2 = "camera2"
# CAM3 = "camera3"

def main():
    # rospy.init_node('data_generator', anonymous=True)
    # bag_name = rospy.get_param('bag_name')
    bag_filepath = os.path.join(BAG_DIR, bag_name)

    bag = bagpy.bagreader(bag_filepath)
    joint_states_data = pd.read_csv(bag.message_by_topic(JOINT_STATES))
    poses_data = pd.read_csv(bag.message_by_topic(POSES))
    # poses_data = pd.read_csv(bag.message_by_topic(POSES+CAM1))
    # poses_data = pd.read_csv(bag.message_by_topic(POSES+CAM2))
    # poses_data = pd.read_csv(bag.message_by_topic(POSES+CAM3))
    
    # csv_dir = os.makedirs(os.path.join(CSV_BASE_DIR, bag_name))

    # joint_states_data.to_csv(os.path.join(csv_dir, "joint_states"))
    # poses_data.to_csv(os.path.join(csv_dir, POSES))

if __name__ == "__main__":
    main()