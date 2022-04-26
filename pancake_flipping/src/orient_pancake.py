import numpy as np
import rospy
from time import sleep
import time

from std_msgs.msg import Int64MultiArray, Float64MultiArray, String, UInt64MultiArray, UInt8MultiArray
from sensor_msgs.msg import JointState
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from ar_track_alvar import AlvarMarkers




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
    def __init__(self, bot_type="wx250s", moving_time=2, accel_time=1, dof=6, target_position_type='front', test_mode = False):

        self.len_pan = 0.14 # length of pan in meters
        self.len_handle = 0.04
        # self.len_to_pan_middle = 0.08

        self.home_joint_position = [0, -1.80, 1.55, 0, 0.55, 0] #home position in radians modified to accomdate for pan

        self.pancake_diam = rospy.get_param("diameter")
        self.pancake_radius = (0.5 * self.pancake_diam)

        if target_position_type == 'front':
            self.joint_positions = [0, -1.30, 1.0, 0, 0.60, 0]
            self.target_translation = [self.len_pan - self.pancake_radius, 0, 0]
        elif target_position_type == 'back':
            self.target_translation = [self.len_handle + self.pancake_radius, 0, 0]
            self.joint_positions = [0, -1.80, 1.55, 0, 0.35, 0]
        else:
            raise NotImplementedError('Invalid target position chosen, please specify front or back of pan')



        self.bot_type = bot_type
        self.dof = dof
        self.test_mode = test_mode

        self.shake_time = 0.75
        self.shake_accel = 0.6

        # self.joint_states_listener = rospy.Subscriber('/' + bot_type + '/joint_states', JointState, self.set_ee_position)
        if test_mode:
            self.bot = InterbotixManipulatorXS(bot_type, "arm", "gripper", moving_time=6, accel_time=3)
        else:
            self.bot = InterbotixManipulatorXS(bot_type, "arm", "gripper", moving_time=moving_time, accel_time=accel_time)

        # rospy.init_node('reorient_pancake_for_flip', anonymous=True)
        # rospy.spin()

        self.end_reorientation = False


    def run_reorientation(self):
        #TODO: do i need to initialize a node when roscore is running??
        # rospy.init_node('reorient_pancake_for_flip', anonymous=True)

        self.major_correction()

        start_time = time.time()
        self.pancake_state_listener = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.reorientation_loop, queue_size=None)
        # rospy.spin()

        while not self.end_reorientation:
            #TODO: i think the code needs to be kept on sleep while subscriber callbacks run??
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

        if self.test_mode:
            print("Starting minor corrective procedure")
            print(f"This marker is associated with: {'global frame' if data.header.frame_id == 0 else 'no frame'}")
            print()

        alvar_markers = data.markers
        alvar_marker = alvar_markers[-1]
        position = alvar_marker.pose.pose.position
        position_vect = [position.x, position.y, position.z]
        if self.check_position(position_vect):
            self.minor_correction()
        else:
            self.close_pancake_topic()
            self.end_reorientation = True


    def major_correction(self):
        """
        Purpose:
            The initial corrective call to tilt pan in desired position direction
        """
        self.bot.arm.set_joint_positions(self.joint_positions)

        if self.test_mode:
            print("Starting reorientation procedure")
            print("Major correction complete")
            print()
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
            print()
            sleep(5)


    def check_position(self, pancake_position, eps=1e-8):
        """
        Parameters:
            translation (np.array): 3 length vector
            eps (float): value for asserting closeness of current position to target

        Return:
            at_positon (bool): true or false value for pancake at correct x position
        """

        ee_pose_matrix = self.bot.get_ee_pose()
        ee_pose_vector = ee_pose_matrix[:3, 3] #get pose vector of the end effector, assuming no rotation and a translation from the same frame as the pancake origin frame

        target_x_y_position = self.target_translation[0:2] + ee_pose_vector[0:2] #translate ee position to front or back of pan

        # keep only x and y position since z position should in practice be static (since pancake lies flat in pan)

        if self.test_mode:
            print("target x, y postion: ", target_x_y_position)
            print("pancake x, y position: ", pancake_position[0:2])
            print("difference between target and pancake position: ", np.diff(target_x_y_position, pancake_position[0:2]))
            print("x, y pancake positions are close: ", np.allclose(target_x_y_position, pancake_position[0:2], atol=eps))
            print()
            sleep(2)

        return np.allclose(target_x_y_position, pancake_position[0:2], atol=eps)
        

if __name__ == "__main__":
    pancake_orienter = PancakeOrienter(test_mode=True)
    print("reorientation succeeded: ", pancake_orienter.run_reorientation())
