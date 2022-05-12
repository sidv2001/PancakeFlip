#!/usr/bin/python3.8
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from time import sleep
import rospy
import logging
from dataset_maker_new import convert_msg_to_tensor
from model_new import RoboticRegression
import numpy as np
from interbotix_xs_msgs.msg import JointGroupCommand
from ar_track_alvar_msgs.msg import SyncedPose

# ______________________________ <CONSTANTS> ___________________________________________________________
# [Waist, Shoulder, ... , Wrist]
HALT = [0, 0, 0, 0, 0, 0]                   # Halt joint velocities
NEW_SLEEP = [0, -1.80, 1.55, 0, 0.55, 0]    # Adjusted sleep position to avoid bending pan
POS = [.25, -0.2 , 0, 0, 0, 1.57/2]         # Tilted pan
VEL1 = [0, 0.4, -0.3, 0, -0.3, 0]           # Move robot forward

counter = 0

CKPT_PATH = "/home/robotics/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/pancake_flipping/src/robot-epoch=160-val_loss=0.11-train_loss=0.19.ckpt"
# CKPT_PATH = "/home/robotics/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/pancake_flipping/src/robot-epoch=67-val_loss=0.02.ckpt"
# CKPT_PATH = "/home/robotics/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/pancake_flipping/src/robot-epoch=142-val_loss=0.17-train_loss=0.14.ckpt"

def pose_cb(synced_pose : SyncedPose, args):
    global counter
    rospy.loginfo('\n\n\n\nCB!\n\n\n\n')
    bot = args[0]   
    model = args[1]
    pub = args[2]
    # cmd = [2 * x for x in cmd]               # Speeding up commands TODO: Why is robot so slow?

    # Use counter to make robot move faster at beginning
    if counter < 10:
        cmd = [x for x in VEL1]
    else:
        x = convert_msg_to_tensor(synced_pose)   # Model Input: Current state
        cmd = model(x).tolist()                  # Model Output: Predicted command
        # cmd = HALT
        cmd = [x for x in cmd]               # Speeding up commands TODO: Why is robot so slow?

    counter += 1
    print(f"Counter: {counter}")

    # Publish model output to separate topic for inspection
    msg = JointGroupCommand()
    msg.name = "arm"
    msg.cmd = cmd
    pub.publish(msg)

    # Write model outputs to robot
    bot.dxl.robot_write_commands("arm", cmd) # Send command for robot to execute


def main():
    # Initialize robot in position control
    bot = InterbotixManipulatorXS("wx250s", "arm", "gripper", moving_time=2, accel_time=2)

    # Go through demonstration.  Home -> Tilted Pan -> Home -> New Sleep 
    # bot.arm.go_to_home_pose()
    # bot.arm.set_joint_positions(POS)
    # bot.arm.go_to_home_pose()
    # bot.arm.set_joint_positions(NEW_SLEEP)
    # bot.arm.go_to_sleep_pose()

    # Set operating mode of robot in velocity control
    # NOTE: Robot sleeps when you switch operating modes 
    bot.dxl.robot_set_operating_modes("group", "arm", "velocity", profile_type="time", profile_velocity=50, profile_acceleration=50)
    model = RoboticRegression.load_from_checkpoint(CKPT_PATH)
    model.eval()
    # rospy.init_node('planner')
    model_cmds_pub = rospy.Publisher("model_commands", JointGroupCommand, queue_size=1)
    synced_pose_sub = rospy.Subscriber("synced_pose", SyncedPose, pose_cb, (bot, model, model_cmds_pub, counter))

    r = rospy.Rate(20) # Rate is in hz
    # Waiting for first synced pose message to be published, which indirectly is waiting for all cameras to be live
    rospy.wait_for_message("synced_pose", SyncedPose) 
    rospy.loginfo('\n\nRunning Learned Velocities!\n\n')
    while not rospy.is_shutdown():
        r.sleep()

if __name__=='__main__':
    main()
