#!/usr/bin/python3.8
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from time import sleep
import rospy
import logging
from dataset_maker import convert_msg_to_tensor
from model import RoboticRegression
import numpy as np
from std_msgs.msg import String
from interbotix_xs_msgs.msg import JointGroupCommand
from ar_track_alvar_msgs.msg import SyncedPose

# ______________________________ <CONSTANTS> ___________________________________________________________
# [Waist, Shoulder, ... , Wrist]
HALT = [0, 0, 0, 0, 0, 0]                   # Halt joint velocities
NEW_SLEEP = [0, -1.80, 1.55, 0, 0.55, 0]    # Adjusted sleep position to avoid bending pan
POS = [.25, -0.2 , 0, 0, 0, 1.57/2]         # Tilted pan
VEL1 = [0, 0.4, -0.3, 0, -0.3, 0]           # Move robot forward

# Bad coding
counter = 0

CKPT_PATH = "/home/robotics/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/pancake_flipping/src/robot-epoch=67-val_loss=0.02.ckpt"
# CKPT_PATH = "/home/robotics/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/pancake_flipping/src/robot-epoch=142-val_loss=0.17-train_loss=0.14.ckpt"

def pose_cb(synced_pose : SyncedPose, args):
    global counter
    rospy.loginfo('\n\n\n\nCB!\n\n\n\n')
    bot = args[0]   
    model = args[1]
    pub = args[2]
    # cmd = [2 * x for x in cmd]               # Speeding up commands TODO: Why is robot so slow?

    # Use counter to make robot move faster at beginning
    if counter < 45:
        cmd = [1.4* x for x in VEL1]
    else:
        x = convert_msg_to_tensor(synced_pose, scalar=1/3)   # Model Input: Current state
        cmd = model(x).tolist()                  # Model Output: Predicted command
        # cmd = HALT
        cmd = [3*x for x in cmd]               # Speeding up commands TODO: Why is robot so slow?

    counter += 1
    print(f"Counter: {counter}")

    # Publish model output to separate topic for inspection
    msg = JointGroupCommand()
    msg.name = "arm"
    msg.cmd = cmd
    pub.publish(msg)


def main():
    rospy.init_node('trigger')
    r = rospy.Rate(30) # Rate is in hz
    first_spacebar_pub = rospy.Publisher("first_spacebar", String, queue_size=1)
    sec_spacebar_pub = rospy.Publisher("second_spacebar", String, queue_size=1)
    first_str = input()
    while(first_str != ' ' and not rospy.is_shutdown()):
        r.sleep()
        first_str = input()
    first_spacebar_pub.publish('True')
    second_str = input()
    while(second_str != ' ' and not rospy.is_shutdown()):
        r.sleep()
        second_str = input()
    sec_spacebar_pub.publish('True')

    rospy.spin()

if __name__=='__main__':
    main()
