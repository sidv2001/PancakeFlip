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
TOPIC = "model_commands"
csv_filename = 'model_commands.csv'
file = os.path.join(BAG_DIR, 'test.bag')
def main():
    print(file)
    bag = bagpy.bagreader(file)
    pd.read_csv(bag.message_by_topic(TOPIC))

if __name__ == "__main__":
    main()