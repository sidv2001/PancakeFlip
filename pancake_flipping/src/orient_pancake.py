import numpy as np
import rospy

from std_msgs.msg import Int64MultiArray, Float64MultiArray, String, UInt64MultiArray, UInt8MultiArray
from sensor_msgs.msg import JointState
from interbotix_xs_modules.arm import InterbotixManipulatorXS


class TopicListener:
    def __init__(self, topic, callback, message_type=Float64MultiArray, node_name='listener'):
        rospy.init_node(node_name, anonymous=True)
        self.joint_subscriber = rospy.Subscriber(topic, message_type, callback, queue_size=1)
        rospy.spin()

class PancakeOrienter:
    def __init__(self, bot_type="wx250s", moving_time=2, accel_time=1, dof=6):
        self.joint_states_listener = TopicListener('/' + bot_type + '/joint_states', self.run_reorientation_loop, message_type=JointState, node_name='joint_state_listener')
        self.bot = InterbotixManipulatorXS(bot_type, "arm", "gripper", moving_time=moving_time, accel_time=accel_time)
        self.dof = dof

    def run_reorientation_loop(self, data):
        """
        Parameters:
            data (sensor_msgs/JointState.msg): Message of string[] name, float64[] position, float64[] velocity, float64[] effort
        """
        joint_states = data.


        pass

    def reorient_pancake(self, target_position):
        pass

    def check_orientation(self, quaternion, translation, target_position=None, eps=1e-6):
        """
        Parameters:
            quaternion (np.array): 4 x 4 quaternion matrix
            translation (np.array): 3 length vector
            target positon (np.array): 4 X 4 quaternion matrix
            eps (float): value for asserting closeness of current position to target

        Return:
            at_positon (bool): true or false value for pancake at postion
        """
        pass

