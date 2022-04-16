from utils import *
import argparse
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument('--bagfile', type=str, default='flip_pancake_1.bag')
parser.add_argument('--directory', type=str, default='/home/robotics/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/examples/interbotix_xsarm_puppet/bag/')
args = parser.parse_args()

def convert_pd_to_velocities(bag_file_pd_data):
    time_differences = np.ediff1d(bag_file_pd_data['Time'])
    # print(bag_file_pd_data)
    columns = bag_file_pd_data.columns
    joint_columns = columns[2:]
    # print(joint_columns)
    pd_data_joints = bag_file_pd_data[joint_columns].to_numpy()
    # print(pd_data_joints.head)
    positions_differences = np.diff(pd_data_joints, axis=0)
    # print(pd_data_joints.shape, positions_differences.shape)
    assert positions_differences.shape == (pd_data_joints.shape[0] - 1, pd_data_joints.shape[1])

    # first_vel_command = positions_differences[0] / time_differences[0]

    velocities = positions_differences / time_differences.reshape(len(time_differences), 1)
    # print(velocities.shape, positions_differences.shape)
    # print(first_vel_command, velocities[0])
    # assert first_vel_command == velocities[0]
    assert velocities.shape == positions_differences.shape

    print(bag_file_pd_data[40:45])
    print(positions_differences[40:45])
    print(time_differences[40:45])
    print(velocities[40:45])

    return time_differences, velocities

def convert_bag_to_vel(bag_file):
    bag_file_pd_data = read_bag_file_to_pd(bag_file)
    time_differences, velocity_commands = convert_to_velocites(bag_file_pd_data)
    return time_differences, velocity_commands






if __name__ == "__main__":
    bag_file_pd_data = read_bag_file_to_pd(args.directory + args.bagfile)
    velocity_commands = convert_pd_to_velocities(bag_file_pd_data)

