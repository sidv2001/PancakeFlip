#!/usr/bin/python3.8
from turtle import pos
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from time import sleep
from std_msgs.msg import String
import rospy
import math
import logging
from dataset_maker_flip_only import convert_msg_to_tensor, convert_msg_to_tensor_vel
from model import RoboticRegressionVel
from model_new import RoboticRegression
from sensor_msgs.msg import JointState
import numpy as np
from interbotix_xs_msgs.msg import JointGroupCommand
from ar_track_alvar_msgs.msg import SyncedPose

# ______________________________ <CONSTANTS> ___________________________________________________________
# VELOCITIES
# [Waist, Shoulder, ... , Wrist]
HALT = [0, 0, 0, 0, 0, 0]                   # Halt joint velocities
NEW_SLEEP = [0, -1.80, 1.55, 0, 0.55, 0]    # Adjusted sleep position to avoid bending pan
POS = [.25, -0.2 , 0, 0, 0, 1.57/2]         # Tilted pan
VEL1 = [0, 0.4, -0.3, 0, -0.3, 0]           # Move robot forward
DROP1 = [0, 0, .6, 0, 0, 0]
RAISE = [0, -.5, -1.75, 0, 0, 0]
DROP2 = [0, .25, 1.125, 0, 0, 0]
RAISE2 = [0, 0, -1, 0, 0, 0]
# RAISE = [-1 * x for x in DROP]
START = [0, 0.4, -0.2, 0, -0.25, 0]         # Move robot to position new data was collected from
START_DURATION =  3.97                         # How long to execute START

# POSITIONS
# POS_SLEEP = 
POS_START =      [0.01073, -0.18407, 0.86129, -0.00153, -0.36355, 0.13038, -0.25003, 0.01633, -0.016334]
POS_PRE_SLEEP =  [0.00920, -1.5,     1.3    , -0.00306,  0.67188, 0.02914, -0.23930, 0.01645, -0.016455] 
POS_SLEEP =      [0.00920, -1.91594, 1.63675, -0.00306,  0.67188, 0.02914, -0.23930, 0.01645, -0.016455]

counter = 0
is_initiated = False

# ____________________________ <MODEL WEIGHTS>___________________________________________________________
# Model trained WITHOUT Velocity on NEW dataset
# CKPT_PATH = "/home/robotics/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/pancake_flipping/src/flip_only_position_1_robot-epoch=299-val_loss=0.1044-train_loss=0.0779.ckpt"
# WITH_VEL = False

# Model trained WITHOUT Velocity on NEW dataset # 2
# CKPT_PATH = "/home/robotics/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/pancake_flipping/src/flip_only_position_2_robot-epoch=399-val_loss=0.1038-train_loss=0.0664.ckpt"
# WITH_VEL = False

# Model trained WITHOUT Velocity on NEW dataset # 3
# CKPT_PATH = "/home/robotics/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/pancake_flipping/src/flip_only_position_3_robot-epoch=395-val_loss=0.0426-train_loss=0.0558-train.ckpt"
# WITH_VEL = False

# Model trained WITHOUT Velocity on NEW dataset # 4
# CKPT_PATH = "/home/robotics/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/pancake_flipping/src/flip_only_position_4_robot-epoch=264-val_loss=0.0889-train_loss=0.0803.ckpt"
# WITH_VEL = False

# Mode trained WITH Velocity on NEW dataset | #1
# CKPT_PATH = "/home/robotics/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/pancake_flipping/src/flip_only_velocity_1_robot-epoch=187-val_loss=0.0137-train_loss=0.0055.ckpt"
# WITH_VEL = True 

# Mode trained WITH Velocity on NEW dataset | #2 | Optimized for training
# CKPT_PATH = "/home/robotics/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/pancake_flipping/src/flip_only_velocity_2_robot-epoch=191-val_loss=0.0088-train_loss=0.0004-train.ckpt"
# WITH_VEL = True 

# Mode trained WITH Velocity on NEW dataset | #3 | Optimized for validation
CKPT_PATH = "/home/robotics/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/pancake_flipping/src/flip_only_velocity_3_robot-epoch=199-val_loss=0.0135-train_loss=0.0045.ckpt"
WITH_VEL = True 

# CKPT_PATH = "/home/robotics/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/pancake_flipping/src/robot-epoch=160-val_loss=0.11-train_loss=0.19.ckpt"
# CKPT_PATH = "/home/robotics/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/pancake_flipping/src/robot-epoch=67-val_loss=0.02.ckpt"
# CKPT_PATH = "/home/robotics/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/pancake_flipping/src/robot-epoch=142-val_loss=0.17-train_loss=0.14.ckpt"

# ____________________________ <CALLBACK>___________________________________________________________
def pose_cb(synced_pose : SyncedPose, args):
    global counter
    global is_initiated
    # rospy.loginfo('\nCB!\n')
    if counter < 6:
        bot = args[0]   
        model = args[1]
        # pub = args[2]
        nos = 9
        pub = args[4]

        # Get model input.  Accounts for if using current velocities or not
        x = convert_msg_to_tensor_vel(synced_pose, scalar=1/nos) if WITH_VEL else convert_msg_to_tensor(synced_pose)
        cmd = model(x).tolist()    # Model Output: Predicted command
        #  initiate only if both 5th cmd is upward motion and 3rd cmd is not extreme
        if(not is_initiated and not (cmd[4] < -0.125 and cmd[4] > -0.2 and abs(cmd[2]) < 0.02)): # "Safety" Guards on Learning
            print(f'[Rejected]: cmd4:{cmd[4]} \t cmd2:{cmd[2]} \t cmd1:{cmd[1]}')
            return
        cmd = [np.sign(x) * 0.2 if abs(x) > 0.2 else x for x in (cmd)]
        print(f'[Accepted]: cmd4:{cmd[4]} \t cmd2:{cmd[2]} \t cmd1:{cmd[1]}')
        is_initiated = True
        cmd = [nos * x if i in (1,2,4) else x for i, x in enumerate(cmd)]
        cmd[4] *= 1.5

        # Publish model output to separate topic for inspection
        # TODO: Are we inspecting?
        msg = JointGroupCommand()
        msg.name = "arm"
        msg.cmd = cmd
        pub.publish(msg)

        # Write model outputs to robot
        # bot.dxl.robot_write_commands("arm", cmd) # Send command for robot to execute
    else:
        exit_pub = args[3]
        exit_pub.publish("True")
    counter += 1

def move_to_position_vel(bot, pos_to_go, time):
    iterations = 2
    for i in range(iterations):
        time = time * (iterations - i) / iterations
        msg = rospy.wait_for_message("/wx250s/joint_states", JointState)
        pos_diff = np.asarray(pos_to_go)[:-3] - np.asarray(msg.position)[:-3]
        vel = pos_diff / time
        bot.dxl.robot_write_commands("arm", vel)
        sleep(time)
    bot.dxl.robot_write_commands("arm", HALT)

# ____________________________ <MAIN>___________________________________________________________
def main():
    global START
    global START_DURATION    
    
    flipped_pub = rospy.Publisher("flipped", String, queue_size=1)
    debug_pub = rospy.Publisher("debug", JointGroupCommand, queue_size=1)
    model_command_pub = rospy.Publisher("/wx250s/commands/joint_group", JointGroupCommand)

    # Initialize robot in position control
    bot = InterbotixManipulatorXS("wx250s", "arm", "gripper", moving_time=2, accel_time=1)

    # Set operating mode of robot in velocity control
    # NOTE: Robot sleeps when you switch operating modes 
    # TODO: Do we use velocity or time for profile_type?
    bot.dxl.robot_set_operating_modes("group", "arm", "velocity", profile_type="time", profile_velocity=0, profile_acceleration=0)

    rospy.wait_for_message("synced_pose", SyncedPose)

    # Move to position we collected data 
    move_to_position_vel(bot, POS_START, 2)
    
    # Load model and put in evaluation mode.  Accounts for if we are using current velocities or not
    model = RoboticRegressionVel.load_from_checkpoint(CKPT_PATH) if WITH_VEL else RoboticRegression.load_from_checkpoint(CKPT_PATH)

    # bot.dxl.robot_write_commands("arm", HALT)
    # rospy.wait_for_message("synced_pose", SyncedPose)

    # Initiate drop and raise
    # bot.dxl.robot_write_commands("arm", DROP1)
    # sleep(.25)
    # bot.dxl.robot_write_commands("arm", RAISE)
    # sleep(.25)
    # bot.dxl.robot_write_commands("arm", DROP2)
    # sleep(.25)

    # Sub to synced_pose and publish commands to model_commands in callback
    model_commands_pub = rospy.Publisher("/wx250s/commands/joint_group", JointGroupCommand, queue_size=1)
    synced_pose_sub = rospy.Subscriber("synced_pose", SyncedPose, pose_cb, (bot, model, model_commands_pub, flipped_pub, model_commands_pub))
    rospy.loginfo('\n\nRunning Learned Velocities!\n\n')

    rospy.wait_for_message("flipped", String)
    bot.dxl.robot_write_commands("arm", DROP2)
    sleep(0.4)
    move_to_position_vel(bot, POS_PRE_SLEEP, 2)
    move_to_position_vel(bot, POS_SLEEP, 2)
    synced_pose_sub.unregister()

    # Keep on running baby.  Can't stop, won't stop. To infinity and beyond.
    # r = rospy.Rate(20) # Rate is in hz
    # while not rospy.is_shutdown():
    #     r.sleep()

if __name__=='__main__':
    str_input = input()
    while(str_input != 'q'):
        counter = 0
        is_initiated = False
        main()
        str_input = input()
