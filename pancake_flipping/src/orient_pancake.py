import numpy as np
import rospy
from time import sleep
import time

from std_msgs.msg import Int64MultiArray, Float64MultiArray, String, UInt64MultiArray, UInt8MultiArray
from sensor_msgs.msg import JointState
from interbotix_xs_modules.arm import InterbotixManipulatorXS

class PancakeState:
    def __init__(self):
        self.quaternion = np.zeros((4, 4))
        self.translation = np.zeros((3, ))

class TopicListener:
    def __init__(self, topic, callback, message_type=Float64MultiArray, node_name='listener'):
        rospy.init_node(node_name, anonymous=True)
        self.joint_subscriber = rospy.Subscriber(topic, message_type, callback, queue_size=1)
        rospy.spin()

class PancakeOrienter:
    def __init__(self, bot_type="wx250s", moving_time=2, accel_time=1, dof=6, target_position_type='front', test_mode = True):

        self.home_position = [0, -1.80, 1.55, 0, 0.55, 0]

        if target_position_type == 'front':
            self.joint_positions = [0, -1.30, 1.0, 0, 0.60, 0]
            self.target_translation = [0.127, 0, 0]
        elif target_position_type == 'back':
            self.target_translation = [0.0762, 0, 0]
            self.joint_positions = [0, -1.80, 1.55, 0, 0.35, 0]
        else:
            raise NotImplementedError('Invalid target position chosen, please specify front or back of pan')



        self.bot_type = bot_type
        self.dof = dof
        self.test_mode = test_mode

        self.shake_time = 0.70
        self.shake_accel = 0.5

        # self.joint_states_listener = rospy.Subscriber('/' + bot_type + '/joint_states', JointState, self.set_ee_position)
        if test_mode:
            self.bot = InterbotixManipulatorXS(bot_type, "arm", "gripper", moving_time=5, accel_time=3)
        else:
            self.bot = InterbotixManipulatorXS(bot_type, "arm", "gripper", moving_time=moving_time, accel_time=accel_time)

        # rospy.init_node('reorient_pancake_for_flip', anonymous=True)
        # rospy.spin()

        self.end_reorientation = False

        self.major_correction()

    def run_reorientation(self):
        #TODO: do i need to initialize a node when roscore is running??
        # rospy.init_node('reorient_pancake_for_flip', anonymous=True)
        start_time = time.time()
        self.pancake_state_listener = rospy.Subscriber('/' + self.bot_type + '/pancake_translation', Float64MultiArray, self.reorientation_loop, queue_size=None)
        # rospy.spin()

        while not self.end_reorientation:
            # i think the code needs to be kept on sleep while subscriber callbacks run
            sleep(1)
            if (time.time() - start_time) > 60:
                return False
        
        return True



    def close_pancake_topic(self):
        self.pancake_state_listener.unregister()

    def set_ee_position(self, data):
        """
        Parameters:
            data (sensor_msgs/JointState.msg): Message of string[] name, float64[] position, float64[] velocity, float64[] effort
        """
        #TODO: what kind of value can we expect to get here?? (x, y, z)?
        self.ee_position = data.position[-1]
        # unsubscribe from joint_states topic, ee position should be static after major correction
        self.joint_states_listener.unregister()
        # open pancake topic to start minor corrective measures
        self.open_pancake_topic()

    def reorientation_loop(self, data):
        """
        Parameters:
            data (sensor_msgs/Float64MultiArray): float64[] data - array representing pancake quaternion
        """

        # pancake_position_quaternion = data.quaternion
        pancake_position_translation = np.array(data.data)
        if self.check_position(pancake_position_translation):
            self.minor_correction()
        else:
            self.pancake_state_listener.unregister()
            self.end_reorientation = True


    def major_correction(self):
        """
        Purpose:
            The initial corrective call to tilt pan in desired position direction
        """
        self.bot.arm.set_joint_positions(self.joint_positions)

        if self.test_mode:
            print("Major correction complete")
            sleep(5)


    def minor_correction(self):
        """
        Purpose:
            runs minor corrective procedure (shaking robot wrist)
        """
        left_tilt = self.joint_positions
        left_tilt[-1] = -0.2
        right_tilt = self.joint_positions
        right_tilt[-1] = 0.2
        
        self.bot.arm.set_joint_positions(left_tilt, moving_time=self.shake_time, accel_time=self.shake_accel)

        if self.test_mode:
            print("Ran left tilt")

        self.bot.arm.set_joint_positions(right_tilt, moving_time=self.shake_time, accel_time=self.shake_accel)

        if self.test_mode:
            print("ran right tilt")
            sleep(5)


    def check_position(self, translation, eps=1e-8):
        """
        Parameters:
            translation (np.array): 3 length vector
            eps (float): value for asserting closeness of current position to target

        Return:
            at_positon (bool): true or false value for pancake at correct x position
        """

        if self.test_mode:
            print("difference between target and actual transaltion from EE: ", np.diff(translation, self.target_translation))
            print("vectors are close: ", np.allclose(translation[0], self.target_translation[0], atol=eps))

        return np.allclose(translation[0], self.target_translation[0], atol=eps)
        

