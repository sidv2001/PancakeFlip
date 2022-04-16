import time
from time import sleep
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from interbotix_perception_modules.armtag import InterbotixArmTagInterface
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface
import json
import argparse
import bagpy
import pandas as pd
# This script uses a color/depth camera to get the arm to find objects and pick them up.
# For this demo, the arm is placed to the left of the camera facing outward. When the
# end-effector is located at x=0, y=-0.3, z=0.2 w.r.t. the 'wx200/base_link' frame, the AR
# tag should be clearly visible to the camera. A small basket should also be placed in front of the arm.
#
# To get started, open a terminal and type 'roslaunch interbotix_xsarm_perception xsarm_perception.launch robot_model:=wx200'
# Then change to this directory and type 'python pick_place.py'

parser = argparse.ArgumentParser()
#playing the file with no arguments passed performs original functionality
parser.add_argument('--from_bag', type=bool, default=False)
parser.add_argument('--from_json', type=bool, default=False)
parser.add_argument('--jsonfile', type=str, default='flip_only.json')
parser.add_argument('--bagfile', type=str, default='flip_only.bag')
parser.add_argument('--bagdirectory', type=str, default='/home/robotics/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/examples/interbotix_xsarm_puppet/bag/')
parser.add_argument('--jsondirectory', type=str, default='/home/robotics/bag_editing/')
parser.add_argument('--move_time', type=int, default=6)
parser.add_argument('--accel_time', type=int, default=3)

args = parser.parse_args()



def main():
    #moved move_time and accel_time to args for testing
    bot = InterbotixManipulatorXS("wx250s", moving_time=args.move_time, accel_time=args.accel_time, gripper_pressure=1.0)
    # bot.gripper.set_pressure(1)
    try:
        #Initialize the arm module along with the pointcloud and armtag modules
        #bot = InterbotixManipulatorXS("wx250s", moving_time=args.move_time, accel_time=args.accel_time)
        bot.gripper.open()
        #set initial arm and gripper pose
        # bot.arm.set_ee_pose_components(x=0.3, z=0.22, pitch=1.4)
        sleep(5)
        bot.gripper.close()
        sleep(5)
        bot.gripper.open()
    except Exception as e:
        print (e)
        print("In except")
        # Go back home
        bot.arm.go_to_sleep_pose()

if __name__=='__main__':
    main()
