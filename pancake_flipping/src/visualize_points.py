import rospy 
from std_msgs.msg import String
import time 
import numpy as np
import math 
from time import sleep 
# from laser_line_extraction.msg import LineSegment 
# from laser_line_extraction.msg import LineSegmentList 
from ar_track_alvar_msgs.msg import AlvarMarkers
# from box_finder_var import *

from interbotix_xs_modules.arm import InterbotixManipulatorXS

from visualization_msgs.msg import Marker 
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3 
from std_msgs.msg import Header, ColorRGBA


def get_point_arrays(bot):
    
    # Pan parameters (meters)
    ee_offset = 0.03 # estimated offset from end of pan handle to ee position
    pan_edge_offset = 0.0125 # Offset from edge of pan to flat inner surface
    len_pan = 0.215    # Length of entire pan
    len_handle = 0.09 # Length of just handle
    
     # Pancake parameters
    pancake_diam = 0.0508
    pancake_radius = (0.5 * pancake_diam)

    # Getting front and back position of pan while accounting for various offsets
    front_position = -ee_offset + len_pan - pan_edge_offset - pancake_radius
    back_position = -ee_offset + len_handle + pan_edge_offset + pancake_radius

    # Homogenized target positions 
    target_front = [front_position, 0, 0, 1]
    target_back = [back_position, 0, 0, 1]
    
    # Transformation from ee frame to base frame
    # Multiply with target positions to get world space coordinates
    M = bot.arm.get_ee_pose() 
    target_front = M @ target_front
    target_back = M @ target_back

    return target_front, target_back, M[:3,3]



# rospy.init_node('box_finder', anonymous=True)

marker_publisher = rospy.Publisher('box_visual', Marker, queue_size=100) 
rospy.sleep(1)

def callback(data, bot):

    # calculate some points...

    ### Visualize the L-shape elements in rviz ###

    show_points_in_rviz(marker_publisher, bot)

def show_points_in_rviz(marker_publisher, bot):
    target_pos_front, target_pos_back, ee_pos = get_point_arrays(bot)

    marker_front = Marker(
                ns='box_finder_front',
                type=Marker.POINTS,
                action=Marker.ADD,
                id=0,
                lifetime=rospy.Duration(),
                pose=Pose(Point(target_pos_front[0], target_pos_front[1], target_pos_front[2]), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.01, 0.01, 0.0),
                header=Header(frame_id='wx250s/base_link'),
                color=ColorRGBA(1.0, 0.0, 0.0, 0.8),
                # lifetime=0,
                points = [Point(0,0,0)],
                frame_locked=False)
    marker_publisher.publish(marker_front)

    marker_back = Marker(
                ns='box_finder_back',
                type=Marker.POINTS,
                action=Marker.ADD,
                id=0,
                lifetime=rospy.Duration(),
                pose=Pose(Point(target_pos_back[0], target_pos_back[1], target_pos_back[2]), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.01, 0.01, 0.0),
                header=Header(frame_id='wx250s/base_link'),
                color=ColorRGBA(0.0, 0.0, 1.0, 0.8),
                # lifetime=0,
                points = [Point(0,0,0)],
                frame_locked=False)
    marker_publisher.publish(marker_back)


    marker_ee = Marker(
                ns='box_finder_ee',
                type=Marker.POINTS,
                action=Marker.ADD,
                id=0,
                lifetime=rospy.Duration(),
                pose=Pose(Point(ee_pos[0], ee_pos[1], ee_pos[2]), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.08, 0.08, 0.0),
                header=Header(frame_id='wx250s/base_link'),
                color=ColorRGBA(1.0, 0.0, 1.0, 0.8),
                # lifetime=0,
                points = [Point(0, 0, 0)],
                frame_locked=False)
    marker_publisher.publish(marker_ee)

def box_finder():

    bot = InterbotixManipulatorXS("wx250s", "arm", "gripper", moving_time=5, accel_time=3)
    # bot.dxl.robot_set_operating_modes("group", "arm", "position", profile_type="time", profile_velocity=5000, profile_acceleration=5000)
    # bot.dxl.robot_set_operating_modes("single", "arm", "current", profile_type="time", profile_velocity=5000, profile_acceleration=5000)
    rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback, callback_args=bot)
    rospy.spin()

if __name__ == '__main__':
    box_finder()