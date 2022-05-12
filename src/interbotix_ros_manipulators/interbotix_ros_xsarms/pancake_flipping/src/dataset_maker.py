from importlib.metadata import files
import os
import numpy as np 
import pandas as pd
import torch
import matplotlib.pyplot as plt





def get_dataset(dataset_name):

    script_dir = os.path.dirname(os.path.realpath(__file__))

    dataset_dir = os.path.join(script_dir, dataset_name)

    files = [f for f in os.listdir(dataset_dir) if os.path.isfile(os.path.join(dataset_dir, f))]
    finX = []
    finY = []
    for file in files:
        if file[-3:] == "csv":
            df = pd.read_csv(os.path.join(dataset_dir, file))
            zerovel = True
            j = 0
            while zerovel != False:
                if df["joint_states.velocity"][j] == "(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)":
                    df = df.drop(j)
                    j += 1
                else: 
                    zerovel = False
            X, Y = create_tensors(df)
            X = X[:-1]   
            Y = Y[1:]  
            finX.append(X)
            finY.append(Y)
    finalX = torch.cat(finX, 0)
    finalY = torch.cat(finY, 0)
    print(finalX.shape, finalY.shape)
    return torch.utils.data.TensorDataset(finalX, finalY)





        
        


def create_tensors(df):
    finX = []
    finY = []
    for index, row in df.iterrows():
        X,Y = convert_pandas_row_to_tensor(row)
        x = X.shape[0]
        y = Y.shape[0]
        X = X.reshape(1, x)
        Y = Y.reshape(1, y)
        finX.append(X)
        finY.append(Y)
    finX = torch.cat(finX, 0)
    finY = torch.cat(finY, 0)
    return finX, finY


def createX(px, py, pz, ox, oy, oz, ow, c1, c2, c3, r1, r2, r3, r4, r5, r6, v1, v2, v3, v4, v5, v6):
    """Converts the following arguments into a tensor

    Args:
        px (float): Avg Pancake Location x
        py (float): Avg Pancake Location y
        pz (float): Avg Pancake Location z
        ox (float): Avg Pancake Orientation x
        oy (float): Avg Pancake Orientation y
        oz (float): Avg Pancake Orientation z
        ow (float): Avg Pancake Orientation w
        c1 (float): Camera 1 Used? (1 or -1)
        c2 (float): Camera 2 Used? (1 or -1)
        c3 (float): Camera 3 Used? (1 or -1)
        r1 (float): Robot Join Angle 1
        r2 (float): Robot Join Angle 2
        r3 (float): Robot Join Angle 3
        r4 (float): Robot Join Angle 4
        r5 (float): Robot Join Angle 5
        r6 (float): Robot Join Angle 6
        v1 (Float): Velocity for Joint Angle 1
        v2 (Float): Velocity for Joint Angle 2
        v3 (Float): Velocity for Joint Angle 3
        v4 (Float): Velocity for Joint Angle 4
        v5 (Float): Velocity for Joint Angle 5
        v6 (Float): Velocity for Joint Angle 6
        
    """

    return torch.tensor([px,py,pz,ox,oy,oz,ow,c1,c2,c3,r1,r2,r3,r4,r5,r6, v1, v2, v3, v4, v5 ,v6])


def createY(v1, v2, v3, v4, v5, v6):
    """_summary_

    Args:
        v1 (Float): Velocity for Joint Angle 1
        v2 (Float): Velocity for Joint Angle 2
        v3 (Float): Velocity for Joint Angle 3
        v4 (Float): Velocity for Joint Angle 4
        v5 (Float): Velocity for Joint Angle 5
        v6 (Float): Velocity for Joint Angle 6
    """
    return torch.tensor([v1, v2, v3, v4, v5 ,v6])

def get_camera(cam_type):
    if cam_type == "wx250s/base_link":
        return 1
    elif cam_type == "-1":
        return -1
    else:
        raise ValueError

def convert_pandas_row_to_tensor(row):
    """_summary_

    Args:
        row (Pandas Row): Pandas Row containing the data

    Returns:
        X: X Tensor
        Y: Y Tensor
    """
    px = row['avg_marker.pose.pose.position.x']
    py = row['avg_marker.pose.pose.position.y']
    pz = row['avg_marker.pose.pose.position.z']
    ox = row['avg_marker.pose.pose.orientation.x']
    oy = row['avg_marker.pose.pose.orientation.y']
    oz = row['avg_marker.pose.pose.orientation.z']
    ow = row['avg_marker.pose.pose.orientation.w']
    cam1 = row['cam1_marker.header.frame_id']
    c1 = get_camera(cam1)
    cam2 = row['cam2_marker.header.frame_id']
    c2 = get_camera(cam2)
    cam3 = row['cam3_marker.header.frame_id']
    c3= get_camera(cam3)
    robot_joint_angles = row["joint_states.position"].split(", ")
    robot_joint_angles[0] = robot_joint_angles[0][1:]
    robot_joint_angles[-1] = robot_joint_angles[-1][:-1]
    fin = list(map(lambda x: float(x), robot_joint_angles))
    r1 = fin[0]
    r2 = fin[1]
    r3 = fin[2]
    r4 = fin[3]
    r5 = fin[4]
    r6 = fin[5]
    robot_joint_velocities = row["joint_states.velocity"].split(", ")
    robot_joint_velocities[0] = robot_joint_velocities[0][1:]
    robot_joint_velocities[-1] = robot_joint_velocities[-1][:-1]
    fin_vel = list(map(lambda x: float(x), robot_joint_velocities)) 
    v1 = fin_vel[0]
    v2 = fin_vel[1]
    v3 = fin_vel[2]
    v4 = fin_vel[3]
    v5 = fin_vel[4]
    v6 = fin_vel[5]
    X = createX(px, py, pz, ox, oy, oz, ow, c1, c2, c3, r1, r2, r3, r4, r5, r6, v1, v2, v3, v4, v5, v6)
    Y = createY(v1, v2, v3, v4, v5, v6)
    return X, Y

def convert_msg_to_tensor(msg, scalar=1):
    """_summary_

    Args:
        msg (ROS msg): Ros msg

    Returns:
        X: X Tensor
        Y: Y Tensor
    """
    px = msg.avg_marker.pose.pose.position.x
    py = msg.avg_marker.pose.pose.position.y
    pz = msg.avg_marker.pose.pose.position.z
    ox = msg.avg_marker.pose.pose.orientation.x
    oy = msg.avg_marker.pose.pose.orientation.y
    oz = msg.avg_marker.pose.pose.orientation.z
    ow = msg.avg_marker.pose.pose.orientation.w
    cam1 = msg.cam1_marker.header.frame_id
    c1 = (1 if cam1 == "wx250s/base_link" else -1)
    cam2 = msg.cam2_marker.header.frame_id
    c2 = (1 if cam2 == "wx250s/base_link" else -1)
    cam3 = msg.cam3_marker.header.frame_id
    c3 = (1 if cam3 == "wx250s/base_link" else -1)
    fin = list(msg.joint_states.position)
    r1 = fin[0]
    r2 = fin[1]
    r3 = fin[2]
    r4 = fin[3]
    r5 = fin[4]
    r6 = fin[5]
    fin_vel = list(msg.joint_states.velocity)
    v1 = fin_vel[0]*scalar
    v2 = fin_vel[1]*scalar
    v3 = fin_vel[2]*scalar
    v4 = fin_vel[3]*scalar
    v5 = fin_vel[4]*scalar
    v6 = fin_vel[5]*scalar
    X = createX(px, py, pz, ox, oy, oz, ow, c1, c2, c3, r1, r2, r3, r4, r5, r6, v1, v2, v3, v4, v5, v6)
    return X


if __name__ == "__main__":
    print(get_dataset("csv"))









    

