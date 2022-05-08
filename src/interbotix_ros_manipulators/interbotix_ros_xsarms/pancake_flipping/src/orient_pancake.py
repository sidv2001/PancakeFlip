import numpy as np
import rospy
from time import sleep
import time
import message_filters

from std_msgs.msg import Int64MultiArray, Float64MultiArray, String, UInt64MultiArray, UInt8MultiArray
from sensor_msgs.msg import JointState
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from ar_track_alvar_msgs.msg import AlvarMarkers




class PancakeOrienter:
    def __init__(self, bot_type="wx250s", moving_time=2, accel_time=1, dof=6, target_position_type='front', test_mode = False, 
                 pancake_diam=0.0508, radius_tolerance=1.3):

        # self.ee_offset = 0.0308 # estimated offset from end of pan handle to ee position

        # self.len_pan = 0.31292 # distance to front position of pan from handle end, 0.31927 pan length - 0.00635 m offset from end of pan to edge pos
        
        # self.len_handle = 0.14364 # distance to back position of pan from handle end, 0.1329 m handle length + 0.00635 m offset from end of handle to edge pos

        # self.front_position = self.len_pan - self.ee_offset
        # self.back_position = self.len_handle - self.ee_offset


        # Pan parameters (meters)
        ee_offset = 0.03 # estimated offset from end of pan handle to ee position
        pan_edge_offset = 0.0125 # Offset from edge of pan to flat inner surface
        len_pan = 0.215    # Length of entire pan
        len_handle = 0.09 # Length of just handle
        
        # Pancake parameters
        # pancake_diam = 0.0508
        pancake_radius = (0.5 * pancake_diam)

        # Getting front and back position of pan while accounting for various offsets
        front_pos = -ee_offset + len_pan - pan_edge_offset - pancake_radius
        back_pos = -ee_offset + len_handle + pan_edge_offset + pancake_radius

        # Homogenized target positions 
        self.front_position = [front_pos, 0, 0, 1]
        self.back_position = [back_pos, 0, 0, 1]


        # self.len_to_pan_middle = 0.08

        self.home_joint_position = [0, -1.80, 1.55, 0, 0.55, 0] #home position in radians modified to accomdate for pan

        try: 
            self.pancake_diam = rospy.get_param("diameter")
        except KeyError:
            self.pancake_diam = pancake_diam
            print(f"rospy param diameter not found. Defaulting to {pancake_diam}")

        self.pancake_radius = (0.5 * self.pancake_diam)

        if target_position_type == 'front':
            self.joint_positions = [0, -1.30, 1.0, 0, 0.70, 0]
            self.target_translation = self.front_position
        elif target_position_type == 'back':
            self.joint_positions = [0, -1, 1.1, 0, -0.4, 0]
            self.target_translation = self.back_position
        else:
            raise NotImplementedError('Invalid target position chosen, please specify front or back of pan')


        self.bot_type = bot_type
        self.dof = dof
        self.test_mode = test_mode

        self.shake_time = 0.4
        self.shake_accel = 0.2

        self.pancake_topic = 'ar_pose_marker'

        # self.joint_states_listener = rospy.Subscriber('/' + bot_type + '/joint_states', JointState, self.set_ee_position)
        if test_mode:
            self.bot = InterbotixManipulatorXS(bot_type, "arm", "gripper", moving_time=5, accel_time=3)
            self.bot.dxl.robot_set_operating_modes("group", "arm", "position", profile_type="time", profile_velocity=5000, profile_acceleration=5000)
            self.bot.dxl.robot_set_operating_modes("single", "arm", "current", profile_type="time", profile_velocity=5000, profile_acceleration=5000)
        else:
            self.bot = InterbotixManipulatorXS(bot_type, "arm", "gripper", moving_time=moving_time, accel_time=accel_time)
            self.bot.dxl.robot_set_operating_modes("group", "arm", "position", profile_type="time", profile_velocity=5000, profile_acceleration=5000)
            self.bot.dxl.robot_set_operating_modes("single", "arm", "current", profile_type="time", profile_velocity=5000, profile_acceleration=5000)

        # rospy.init_node('reorient_pancake_for_flip', anonymous=True)
        # rospy.spin()

        self.end_reorientation = False
        self.reorientation_succeeded = False
        self.sleep = False


    def run_reorientation(self):
        #TODO: do i need to initialize a node when roscore is running??
        # rospy.init_node('reorient_pancake_for_flip', anonymous=True)

        self.major_correction()

        self.start_time = time.time()
        self.open_pancake_topic()
        #TODO: make subscriber 
        # rospy.spin()

        print("instantiate listener")

        # while not self.end_reorientation:
        #     #TODO: i think the code needs to be kept on sleep while subscriber callbacks run??
        #     sleep(1)
        #     if (time.time() - start_time) > 60:
        #         return False
        
        while not self.end_reorientation:
            rospy.sleep(1.)
        
        self.bot.arm.set_joint_positions(self.home_joint_position)

        return self.reorientation_succeeded

    def open_pancake_topic(self):
        self.pancake_state_listener = rospy.Subscriber(self.pancake_topic, AlvarMarkers, callback=self.reorientation_loop, callback_args=self.pancake_topic, queue_size=None)


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

    def reorientation_loop(self, data, topic):
        """
        Parameters:
            data (sensor_msgs/Float64MultiArray): float64[] data - array representing pancake quaternion
        """

        self.close_pancake_topic()

        if time.time() - self.start_time > 20:
            self.end_reorientation = True
            return

        alvar_markers = data.markers
        camera = data.header.frame_id
        self.seq = data.header.seq

        if len(alvar_markers) == 0 or camera != 'camera2':
            # print(f"{data.header.frame_id} found no alvar markers")
            self.open_pancake_topic()
            return

        if self.test_mode: 
            # print("Starting minor corrective procedure")
            # print("camera: ", data.header.frame_id)
            # print("seq: ", data.header.seq)
            # print("topic: ", topic)
            # print(f"This marker is associated with: {'global frame' if data.header.frame_id == 0 else 'no frame'}")
            # print("alvarMarkers time stamp list: ", [alvar_mark.header.stamp.secs for alvar_mark in alvar_markers])
            # print("alvarMarkers frame id list: ", [alvar_mark.header.frame_id for alvar_mark in alvar_markers])
            # print()
            pass
        alvar_marker = alvar_markers[-1]
        position = alvar_marker.pose.pose.position
        position_vect = np.array([position.x, position.y, position.z])
        if not self.check_position(position_vect): #TODO: check that all cameras are getting a true value in closeness
            self.minor_correction()
        else:
            self.close_pancake_topic()
            self.reorientation_succeeded = True
            self.end_reorientation = True
        self.open_pancake_topic()


    def major_correction(self):
        """
        Purpose:
            The initial corrective call to tilt pan in desired position direction
        """
        self.bot.arm.set_joint_positions(self.joint_positions)
        # self.bot.arm.set_joint_positions(self.flattened_joint_pos)

        if self.test_mode:
            print("Starting reorientation procedure")
            print("Major correction complete")
            print()


    def minor_correction(self):
        """
        Purpose:
            runs minor corrective procedure (shaking robot wrist)
        """
        print("run minor correction")
        left_tilt = self.joint_positions.copy()
        up = self.joint_positions.copy()
        up[-2] = up[-2] - 0.15
        left_tilt[-1] = -0.25
        # left_tilt[-2] = left_tilt[-2] - 0.05
        right_tilt = self.joint_positions.copy()
        down = self.joint_positions.copy()
        down[-2] = down[-2] + 0.15
        right_tilt[-1] = 0.25
        # right_tilt[-2] = right_tilt[-2] + 0.05
        for i in range(2):
            self.bot.arm.set_joint_positions(left_tilt, moving_time=self.shake_time, accel_time=self.shake_accel)

            if self.test_mode:
                print("Ran left tilt")


            self.bot.arm.set_joint_positions(right_tilt, moving_time=self.shake_time, accel_time=self.shake_accel)

            if self.test_mode:
                print("ran right tilt")

        for i in range(2):
            self.bot.arm.set_joint_positions(up)
            self.bot.arm.set_joint_positions(down)
        self.bot.arm.set_joint_positions(self.joint_positions)            



    def check_position(self, pancake_position, eps=1e-2):
        """
        Parameters:
            translation (np.array): 3 length vector
            eps (float): value for asserting closeness of current position to target

        Return:
            at_positon (bool): true or false value for pancake at correct x position
        """

        M = self.bot.arm.get_ee_pose() 
        target_position = M @ self.target_translation
        target_position = target_position[0:3]

        if self.test_mode:
            print("target x, y postion: ", target_position)
            print("pancake x, y position: ", pancake_position)
            print("difference between target and pancake position: ", np.subtract(target_position, pancake_position))
            print("x, y pancake positions are close: ", np.allclose(target_position[0:2], pancake_position[0:2], atol=eps))
            print()
            sleep(2)

        return np.allclose(target_position[0:2], pancake_position[0:2], atol=eps)
        

if __name__ == "__main__":
    print("start small pancake reorient")
    pancake_orienter = PancakeOrienter(test_mode=True, target_position_type='back')
    print("small pancake reorientation succeeded: ", pancake_orienter.run_reorientation())
    print()
    # print("Sleeping for 5 seconds, please replace small pancake with large")
    # print()
    # sleep(5)
    # print("start large pancake reorient")
    # pancake_orienter = PancakeOrienter(test_mode=True, pancake_diam=0.0762)
    # print("large pancake reorientation succeeded: ", pancake_orienter.run_reorientation())

