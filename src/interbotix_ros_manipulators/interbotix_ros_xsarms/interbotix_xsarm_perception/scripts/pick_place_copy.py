import time
from time import sleep
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from interbotix_perception_modules.armtag import InterbotixArmTagInterface
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface
import json
import argparse
import bagpy
import pandas as pd
import numpy as np
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
parser.add_argument('--move_time', type=float, default=6.0)
parser.add_argument('--accel_time', type=float, default=3.0)

args = parser.parse_args()



def main():
    #moved move_time and accel_time to args for testing
    bot = InterbotixManipulatorXS("wx250s", moving_time=args.move_time, accel_time=args.accel_time, gripper_pressure=1.0)
    # bot.gripper.set_pressure(1)
    try:
        #Initialize the arm module along with the pointcloud and armtag modules
        #bot = InterbotixManipulatorXS("wx250s", moving_time=args.move_time, accel_time=args.accel_time)
        pcl = InterbotixPointCloudInterface()
        bot.gripper.open(delay=1.0)
        #set initial arm and gripper pose
        # bot.arm.set_ee_pose_components(x=0.3, z=0.22, pitch=1.4)
        #sleep(15)
        # print('Successfully set gripper pressure')
        #bot.gripper.close()

        # # Pick up bottle
        # print("Picking up bottle")
        # bot.arm.set_ee_pose_components(x=.4, z=.35, pitch=1)
        # bot.arm.set_ee_pose_components(x=0.3, z=0.23, pitch=1.4)

        # bot.gripper.open()
        # # Go back home
        # bot.arm.go_to_sleep_pose()
        # print ('sleeping for 2')
        # sleep(2)
        # print('done waiting.')
        # get the cluster positions
        # sort them from max to min 'x' position w.r.t. the 'wx200/base_link' frame
        
        
        joint_positions = []
        if args.from_bag:
            #read specified bag file and convert positions to list 
            bag = bagpy.bagreader(args.bagdirectory + args.bagfile)
            message_by_topic = bag.message_by_topic('/wx250s/commands/joint_group')
            message_by_topic = pd.read_csv(message_by_topic)
            columns = message_by_topic.columns

            joint_columns = message_by_topic.loc[:, columns[2:]]
            joint_positions = np.array(joint_columns.values.tolist())
        elif args.from_json:
            #load list of positions from json file produced with edit_bags.py
            with open(args.jsondirectory + args.jsonfile) as f:
                joint_positions = np.array(json.load(f))


        # success, clusters = pcl.get_cluster_positions(ref_frame="wx250s/base_link", sort_axis="x", reverse=True)
        # print(f'size of clusters {len(clusters)}')
        # #print(joint_positions)

        # for cluster in clusters:
        #     x, y, z = cluster["position"]
        #     bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=0.3)
        #     bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.015, pitch=0.3)
        #     bot.gripper.close(delay=1.0)
            # sleep(3)
        if len(joint_positions) > 0:
            # if a bag file or json file was passed to the code, run through positions, otherwise perform default
            import subprocess
            if args.from_bag:
                print('Moving to first position in bag')
                bot.arm.set_joint_positions(joint_positions[0])
                print('playing bag')
                #TODO: playing bag stops halfway through and does not allow further commands on the bot object
                # err_file = open('pick_place_std_out/pick_place_err.txt', 'w')
                # out_file = open('pick_place_std_out/pick_place_out.txt', 'w')
                # player_proc = subprocess.run(['rosbag', 'play', args.bagfile], cwd=args.bagdirectory, stderr=err_file, stdout=out_file)
                # print(player_proc)
                #sleep(30)
                cut_positions = [pos for i, pos in enumerate(joint_positions) if i % 100 == 0]
                for i, position in enumerate(cut_positions):
                    bot.arm.set_joint_positions(position)
                # bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.015, pitch=1.4)

            elif args.from_json:
                #TODO: json support might not be not necessary??
                num_positions = len(joint_positions)
                print(num_positions)
                #TODO: cutting is being tuned specifically for the flip_only.bag file right now. need to find general method
                # getting every 50th joint command because performing all moves is very slow (timing of bag file commands too fast for this method to keep up)
                #cut_positions = [pos for i, pos in enumerate(joint_positions[20:num_positions-70]) if i % 50 == 0]
                # flip_pt = np.argmin(joint_positions[:, 4]) - 1
                # print(flip_pt)
                start_time = time.time()
                for i, position in enumerate(joint_positions):
                    bot.arm.set_joint_positions(position)
                end_time = time.time()

                print("commands over time: ", (num_positions/(end_time - start_time)))
                    # if i == flip_pt:
                    #     bot.gripper.open()

                # bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.015, pitch=1.4)
        else:
            # bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=1.4)
            # bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.015, pitch=1.4)
            # bot.gripper.open()
            # bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=1.4)
            pass
        # bot.gripper.open()
        # Go back home
        # bot.arm.go_to_sleep_pose()


    except Exception as e:
        print (e)
        print("In except")
        # Go back home
        bot.arm.go_to_sleep_pose()

if __name__=='__main__':
    main()
