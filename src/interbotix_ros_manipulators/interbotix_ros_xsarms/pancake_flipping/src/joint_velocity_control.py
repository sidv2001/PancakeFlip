from interbotix_xs_modules.arm import InterbotixManipulatorXS
from time import sleep
import rospy
import logging
from pos_2_vel import convert_bag_to_vel

# parser = argparse.ArgumentParser()
# parser.add_argument('--bagfile', type=str, default='slow_test.bag')
# parser.add_argument('--directory', type=str, default='/home/robotics/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/examples/interbotix_xsarm_puppet/bag/')
# args = parser.parse_args()

# ______________________________ <CONSTANTS> ___________________________________________________________
# [Waist, Shoulder, ... , Wrist]
NEW_HOME = [0, -1.80, 1.55, 0, 0.55, 0]
POS = [-.25, 0 , 0, 0, 0, 1.57]
# These are set through trial and error. Nothing special about these particular values
# Notice that VEL1 = -VEL2. Running each of these joint velocites for the same amount of time
# will cancel each other out.
VEL1 = [0, 0.4, -0.4, 0, 0, -0.1] 
VEL2 = [0, -0.4 , 0.4, 0, 0, 0.1]
DURATION = 5 # seconds
EPSILON = 0.03 # seconds
HALT = [0, 0, 0, 0, 0, 0]


# This script tests switching between position and velocity control
# TODO(Jonathan):   Investigate why "explicitly" using position control yields VERY high speeds
#                   Directions may be:
#                       1) moving_time and accel_time are both really small. Why? 
#                       2) Notice that using a freshly initialized "InterbotixManipulatorXS" class
#                          does not result in these high speeds. What is being done in "init" that 
#                          is not being done in "robot_set_operating_modes"

def main():
    bot = InterbotixManipulatorXS("wx250s", "arm", "gripper", moving_time=4, accel_time=2)
    # Uncomment the following line for verbose info about initial configuration
    # print(bot.dxl.srv_get_info("group", "arm"))

    # Position Control
    bot.arm.go_to_home_pose()
    sleep(100000)
    bot.arm.set_joint_positions(POS)
    bot.arm.go_to_home_pose()
    
    bot.arm.set_joint_positions(NEW_HOME)

    # # Velocity Control
    bot.dxl.robot_set_operating_modes("group", "arm", "velocity", profile_type="time")
    rospy.loginfo(f'Running VEL1 for {DURATION} seconds.')
    bot.dxl.robot_write_commands("arm", VEL1)
    sleep(DURATION)
    rospy.loginfo(f'Running VEL2 for {DURATION-EPSILON} seconds.')
    bot.dxl.robot_write_commands("arm", VEL2)
    sleep(DURATION-EPSILON)
    bot.dxl.robot_write_commands("arm", HALT)

    # time_differences, velocites = convert_bag_to_vel(args.directory, args.bag_file)

    # for time, vel in zip(time_differences, velocites):
    #     bot.dxl.robot_write_commands("arm", vel)
    #     sleep(time)

    # bot.dxl.robot_write_commands("arm", HALT)

    # # TODO(Jonathan): Investigate necessity of the following line before finishing
    # #   * Result 1: Is necessary! Perhaps state is persistent across multiple runs
    bot.dxl.robot_set_operating_modes("group", "arm", "position", profile_type="time", profile_velocity=5000, profile_acceleration=5000)
    # sleep(4)
    bot.arm.go_to_home_pose()
    # sleep(5)
    bot.arm.go_to_sleep_pose()

if __name__=='__main__':
    main()
