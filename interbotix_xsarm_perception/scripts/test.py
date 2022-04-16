import time
from time import sleep
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from interbotix_perception_modules.armtag import InterbotixArmTagInterface
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface
from interbotix_xs_msgs.msg import *
from interbotix_xs_msgs.srv import *
import json
import argparse
import bagpy
import pandas as pd
import numpy as np

parser = argparse.ArgumentParser()
#playing the file with no arguments passed performs original functionality
parser.add_argument('--move_time', type=float, default=6.0)
parser.add_argument('--accel_time', type=float, default=3.0)

args = parser.parse_args()



def main():
    bot = InterbotixManipulatorXS("wx250s", moving_time=args.move_time, accel_time=args.accel_time, gripper_pressure=1.0)
    vel_op_mode = OperatorModes()
    bot.srv_set_op_modes()
    pcl = InterbotixPointCloudInterface()
    bot.gripper.open(delay=1.0)
    bot.arm.set_ee_pose_components(x=0.3, z=0.22, pitch=1.4)
    sleep(5)
    bot.arm.go_to_sleep_pose()

if __name__=='__main__':
    main()
