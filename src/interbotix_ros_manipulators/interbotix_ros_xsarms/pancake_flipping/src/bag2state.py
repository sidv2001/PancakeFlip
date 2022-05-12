#!/usr/bin/env python  
import utils 
import bagpy
import pandas as pd 
import json
import os
import glob
import shutil

PKG_DIR = '/home/robotics/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/pancake_flipping'
BAG_DIR = os.path.join(PKG_DIR, 'bag')
CSV_BASE_DIR = os.path.join(PKG_DIR, 'csv')

bag_pattern = '*.bag'
POSES = "/synced_pose"
csv_filename = 'synced_pose.csv'

def main():
    for file in glob.glob(os.path.join(BAG_DIR, bag_pattern)):
        print(file)
        bag = bagpy.bagreader(file)
        pd.read_csv(bag.message_by_topic(POSES))
    for bag_data_dir_name in os.listdir(BAG_DIR):
        bag_data_dir = os.path.join(BAG_DIR, bag_data_dir_name)
        if os.path.isdir(bag_data_dir):
            print(bag_data_dir)
            clean_basename = os.path.basename(os.path.normpath(bag_data_dir)) + '.csv'
            shutil.move(os.path.join(bag_data_dir, csv_filename), os.path.join(CSV_BASE_DIR, clean_basename))

if __name__ == "__main__":
    main()