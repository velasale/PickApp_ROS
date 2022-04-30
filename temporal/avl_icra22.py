#!/usr/bin/env python
import math
import sys
import copy
import rospy
import time
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from random import random
import os
import subprocess, shlex, psutil

from math import pi

import std_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler

from std_msgs.msg import String
import csv


## END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class AppleProxyExperiment(object):

    def __init__(self):
        super(AppleProxyExperiment, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('apple_proxy_experiment', anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:

        # group_name = "panda_arm"
        group_name = "manipulator"
        # group_name = "endeffector"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        succes_or_failure_publilsher = rospy.Publisher('/success_or_failure', std_msgs.msg.String, queue_size=20)
        # this topic is published to at the start of major events in each trial and is recorded to the rosbag(0-beginnig of close hand, 1-beginning of apple retrieval)
        event_publisher = rospy.Publisher('/apple_trial_events', std_msgs.msg.Int32, queue_size=20)

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print
        "============ Planning frame: %s" % planning_frame

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print
        "============ End effector link: %s" % eef_link

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print
        "============ Available Planning Groups:", robot.get_group_names()

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print
        "============ Printing robot state"
        print
        robot.get_current_state()
        print
        ""
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.success_or_failure_publisher = succes_or_failure_publilsher
        self.event_publisher = event_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

        # Parameters of the Experiment
        self.gripper_height = 0.132  # Distance from the middle of the fingers to the TCP (Tool Center Point)
        self.delta_z = 0.045  # Distance from the TCP to the center of the apple. ## distance from outside of apple to center of apple

        self.apple_pos_x = 0.487  # Apple position in x-axis [m]
        self.apple_pos_y = -.091  # Apple position in y-axis [m]
        self.apple_pos_z = 0.725 - self.delta_z  # Apple position in z-axis [m] : Height from the + Distance from table to,

        self.initial_roll_angle = 0
        self.initial_pitch_angle = 0
        self.initial_yaw_angle = 0

        self.initial_roll = (pi / 180) * self.initial_roll_angle  # Roll of the initial ideal position [deg]
        self.initial_pitch = (pi / 180) * self.initial_pitch_angle  # Pitch of the initial ideal position [rad]
        self.initial_yaw = (pi / 180) * self.initial_yaw_angle  # Yaw of the initial ideal position [deg]

        # To be measured from drooping fingers in 90 degree position over apple
        self.max_noise_x = 2.5 / 100
        self.max_noise_y = 2.5 / 100
        self.max_noise_z = 1.15 / 100

        # percent noise should be entered in decimal eg 90% = .90
        # negative percentage allowed
        # used for specific amount of noise in each axis rather than a random combination
        self.percent_noise_X =  0
        self.percent_noise_Y = 0
        self.percent_noise_Z = 0  # -.5 seems like its the lowest it can go bc it starts to crash into the apple

        self.noise_x = (self.max_noise_x * self.percent_noise_X)# * random()) - ((self.max_noise_x * self.percent_noise_X)/2)
        self.noise_y = (self.max_noise_y * self.percent_noise_Y)#  * random()) - ((self.max_noise_y * self.percent_noise_Y)/2)
        self.noise_z = (self.max_noise_z * self.percent_noise_Z)#   * random()) - ((self.max_noise_x * self.percent_noise_Z)/2)

        # updated in the add angular noise function to use in the naming function noist_toString
        self.total_roll = self.initial_roll
        self.total_pitch = self.initial_pitch
        self.total_yaw = self.initial_yaw

        # this is updated in the add angular noise method and then used to name the file,
        self.roll_noise = 0
        self.pitch_noise = 0
        self.yaw_noise = 0

        # ideal starting position of apple before goes to pick the apple without noise
        # updatd in go_to_starting_position()
        self.ideal_starting_x = 0
        self.ideal_starting_y = 0
        self.ideal_starting_z = 0

        # ideal position of hand when it goes to pick apple with no noise
        # updated in go_to_pick_apple()
        self.ideal_picking_x = 0
        self.ideal_picking_y = 0
        self.ideal_picking_z = 0

        # actual picking position with noise
        # updated in go_to_pick_apple()
        self.picking_pos_x = 0
        self.picking_pos_y = 0
        self.picking_pos_z = 0

        # this was made so the file name could also include the angular noise by knowing the angular noise to be applied at the instation of the object
        self.noise_range_pitch = 0 #(30 * random()) - (30 / 2)
        self.noise_range_yaw = 0
        self.noise_range_roll = 0 #(30 * random()) - (30 / 2)

        # trial number is used later in the program to make sure the csv and the corresponding bag file have the same pick #
        self.trial_number = 0

    def go_to_home(self):
        current_joints = self.move_group.get_current_joint_values()
        print("Initial Joints State: ", current_joints)

        # Initial / Default joint values
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = - pi / 2
        joint_goal[1] = 0
        joint_goal[2] = - pi * 145 / 180
        joint_goal[3] = - pi * 3 / 4
        joint_goal[4] = - pi / 2
        joint_goal[5] = 0

        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()

        # Print for debugging:
        current_joints = self.move_group.get_current_joint_values()
        print("Final Joints State: ", current_joints)

        return all_close(joint_goal, current_joints, 0.01)

    def go_to_prelim_start(self):
        """ This function is to avoid the robot from travelling around weird points"""

        current_joints = self.move_group.get_current_joint_values()
        print("Initial Joints State: ", current_joints)

        # The following are the initial joints positions
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = - pi / 6
        joint_goal[1] = - 77 * pi / 180
        joint_goal[2] = + 20 * pi / 180
        joint_goal[3] = - 40 * pi / 180
        joint_goal[4] = - pi / 2
        joint_goal[5] = 0

        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()

        # Print for debugging:
        current_joints = self.move_group.get_current_joint_values()
        print("Final Joints State: ", current_joints)

        return all_close(joint_goal, current_joints, 0.01)

    def go_to_starting_position(self):
        """This function takes the gripper to the IDEAL starting position, before adding noise"""
        current_pose = self.move_group.get_current_pose().pose
        print("Current End Tool's Pose: ", current_pose)

        pose_goal = self.move_group.get_current_pose().pose

        # resets the class variables back to zero, this is for naming each bag file
        roll_angle = self.initial_roll
        pitch_angle = self.initial_pitch
        yaw_angle = self.initial_yaw

        quaternion = quaternion_from_euler(roll_angle, pitch_angle, yaw_angle)

        pose_goal.orientation.x = quaternion[0]
        pose_goal.orientation.y = quaternion[1]
        pose_goal.orientation.z = quaternion[2]
        pose_goal.orientation.w = quaternion[3]

        # Cartesian adjustment of the gripper's position, after PITCH rotation so the palm's normal vector points towards the apple
        pose_goal.position.x = self.apple_pos_x - (self.gripper_height + self.delta_z) * math.cos(pitch_angle)
        pose_goal.position.y = self.apple_pos_y  # no movement in y so this becomes a 2d problem between x and z axis
        pose_goal.position.z = self.apple_pos_z - (self.gripper_height + self.delta_z) * (1 - math.sin(pitch_angle))

        self.ideal_starting_x = pose_goal.position.x
        self.ideal_starting_y = pose_goal.position.y
        self.ideal_starting_z = pose_goal.position.z

        # Cartesian adjustment of the gripper's position, after YAW rotation, so the palm's normal vector points towards the apple
        self.pos_x = pose_goal.position.x
        self.pos_z = pose_goal.position.z

        self.move_group.set_pose_target(pose_goal)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()

        # Print for debugging:
        current_pose = self.move_group.get_current_pose().pose
        print("Final End Tool's Pose: ", current_pose)

        return all_close(pose_goal, current_pose, 0.01)

    def add_angular_noise(self):
        pose_goal = self.move_group.get_current_pose().pose
        print("Current End Tool's Pose: ", pose_goal)

        # --- Step 1: Add noise to the Initial Pose
        # Add noise to the ROLL Angle
        # roll_noise_deg = noise_range_roll * random() - noise_range / 2
        # roll_noise_deg = noise_range_roll / 2  this one is used if you want to adjust the angle in the main script
        roll_noise_deg = self.noise_range_roll  # this one is used for the data collection bc it allows for the name of the file to include the angular noise bc the noise would have been set from the start and not in the main script
        roll_noise_rad = roll_noise_deg * pi / 180
        roll_angle = self.initial_roll + roll_noise_rad
        self.total_roll = roll_angle

        # Add noise to the PITCH Angle
        # pitch_noise_deg = noise_range_pitch * random() - noise_range / 2
        # pitch_noise_deg = noise_range_pitch / 2  this one is used if you want to adjust the angle in the main script
        pitch_noise_deg = self.noise_range_pitch
        pitch_noise_rad = pitch_noise_deg * pi / 180
        pitch_angle = self.initial_pitch + pitch_noise_rad
        self.total_pitch = pitch_angle

        # Add noise to the YAW Angle
        # yaw_noise_deg = noise_range_yaw * random() - noise_range / 2
        # yaw_noise_deg = noise_range_yaw / 2  this one is used if you want to adjust the angle in the main script
        yaw_noise_deg = self.noise_range_yaw
        yaw_noise_rad = yaw_noise_deg * pi / 180
        yaw_angle = self.initial_yaw + yaw_noise_rad
        self.total_yaw = yaw_angle

        print('angular noises = ', roll_noise_deg, pitch_noise_deg, yaw_noise_deg)

        # --- Step2: Obtain the quaternion
        quaternion = quaternion_from_euler(roll_angle, pitch_angle, yaw_angle)
        pose_goal.orientation.x = quaternion[0]
        pose_goal.orientation.y = quaternion[1]
        pose_goal.orientation.z = quaternion[2]
        pose_goal.orientation.w = quaternion[3]

        # --- Step 3: Adjust the EndTool effector coordinates after the noise was added
        pose_goal.position.x = self.apple_pos_x - (self.gripper_height + self.delta_z) * math.cos(pitch_angle)
        pose_goal.position.y = self.apple_pos_y
        pose_goal.position.z = self.apple_pos_z - (self.gripper_height + self.delta_z) * (1 - math.sin(pitch_angle))

        self.pos_x = pose_goal.position.x
        self.pos_z = pose_goal.position.z

        # --- Step3: Perform the movement with moveit!
        self.move_group.set_pose_target(pose_goal)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        # Print for debugging:
        current_pose = self.move_group.get_current_pose().pose
        print("Final End Tool's Pose after adding Noise: ", current_pose)

        # Save this pose in a global variable, to come back to it after picking the apple
        self.previous_pose = pose_goal

        return all_close(pose_goal, current_pose, 0.01)

    def go_to_pick_apple(self):
        pose_goal = self.move_group.get_current_pose().pose

        self.ideal_picking_x = self.pos_x + self.delta_z * math.cos(self.total_pitch)
        self.ideal_picking_y = self.apple_pos_y
        self.ideal_picking_z = self.pos_z - self.delta_z * math.sin(self.total_pitch)

        # Pose Position
        pose_goal.position.x = self.pos_x + self.delta_z * math.cos(self.total_pitch) + self.noise_x
        pose_goal.position.y = self.apple_pos_y + self.noise_y
        pose_goal.position.z = self.pos_z - self.delta_z * math.sin(self.total_pitch) + self.noise_z

        self.picking_pos_x = pose_goal.position.x
        self.picking_pos_y = pose_goal.position.y
        self.picking_pos_z = pose_goal.position.z

        self.move_group.set_pose_target(pose_goal)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose

        # Returns a check if the current pose is within a certain tolerance from the pose_goal
        return all_close(pose_goal, current_pose, 0.01)

    def retrieve(self):
        # Set the initial starting position as the goal pose.
        pose_goal = self.previous_pose

        self.move_group.set_pose_target(pose_goal)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose

        # Returns a check if the current pose is within a certain tolerance from the pose_goal
        return all_close(pose_goal, current_pose, 0.01)

    def closeHandService(self):
        os.system("rosservice call /applehand/close_hand")

    def openHandService(self):
        os.system("rosservice call /applehand/open_hand")

    def relaxHandService(self):
        os.system("rosservice call /applehand/relax_hand")

    def noise_toString(self):
        num_formatter = "{0:.2f}"
        output = "_noiseX_" + str(num_formatter.format(self.percent_noise_X)) + "_noiseY_" + str(
            num_formatter.format(self.percent_noise_Y)) + "_noiseZ_" + str(num_formatter.format(self.percent_noise_Z)) \
                 + "_InitPitch_" + str(num_formatter.format(self.initial_pitch_angle)) + "_InitYaw_" + str(
            num_formatter.format(self.initial_yaw_angle)) + "_InitRoll_" + str(
            num_formatter.format(self.initial_roll_angle)) + \
                 "_PitchNoise_" + str(num_formatter.format(self.noise_range_pitch)) + "_RollNoise_" + str(
            num_formatter.format(self.noise_range_roll)) + "_YawNoise_" + str(
            num_formatter.format(self.noise_range_yaw))
        return output

    def success_or_failure(self, sof):
        self.success_or_failure_publisher.publish(sof)

    def publish_event(self, event):
        self.event_publisher.publish(event)

    def write_csv(self, data):
        header = ["stem_orientation", "f0_proximal", "f0_distal", "f1_proximal", "f1_distal", "f2_proximal",
                  "f2_distal", "slip", "drop", "stem_moved",
                  "success_or_failure", "apple_x", "apple_y", "apple_z", "hand_ideal_starting_pos_x",
                  "hand_ideal_starting_pos_y",
                  "hand_ideal_starting_pos_z", "hand_ideal_starting_pos_yaw", "hand_ideal_starting_pos_pitch",
                  "hand_ideal_starting_pos_roll", "hand_picking_pos_ideal_x", "hand_picking_pos_ideal_y",
                  "hand_picking_pos_ideal_z ", "hand_picking_pos_with_noise_x", "hand_picking_pos_with_noise_y",
                  "hand_picking_pos_with_noise_z",
                  "noise_x", "noise_y", "noise_z", "noise_yaw",
                  "noise_pitch", "noise_roll"]
        csv_name = "/home/avl/ur_ws/src/apple_proxy/bag_files/pick" + str(
            self.get_trial()) + self.noise_toString() + ".csv"

        with open(csv_name, 'wb') as f:
            writer = csv.writer(f)

            # write the header
            writer.writerow(header)

            # write the actual data
            writer.writerow(data)

    def get_pitch(self):
        return self.initial_pitch_angle

    def update_trial_num(self, trial_num):
        self.trial_number = trial_num

    def get_trial(self):
        return self.trial_number

    def get_apple_pose(self):
        apple_pos = [self.apple_pos_x, self.apple_pos_y, self.apple_pos_z]
        return apple_pos

    def get_noise_added(self):
        noise = [self.noise_x, self.noise_y, self.noise_z, self.noise_range_yaw, self.noise_range_pitch,
                 self.noise_range_roll]
        return noise

    def hand_ideal_starting_position_data(self):
        hand_data = [self.ideal_starting_x, self.ideal_starting_y, self.ideal_starting_z, self.initial_yaw_angle,
                     self.initial_pitch_angle, self.initial_roll_angle]
        return hand_data

    def hand_ideal_picking_position_data(self):
        hand_data = [self.ideal_picking_x, self.ideal_picking_y, self.ideal_picking_z]
        return hand_data

    def hand_picking_position_with_noise(self):
        hand_data = [self.picking_pos_x, self.picking_pos_y, self.picking_pos_z]
        return hand_data


def main():
    try:
        print("")
        print("----------------------------------------------------------")
        print(" Apple Proxy Experiment with UR5 and Robotiq 2f-85 gripper")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")
        print("============ Press `Enter` to begin the Experiment by setting up the moveit_commander ======")
        raw_input()
        apple_proxy_experiment = AppleProxyExperiment()

        print("============ Press `Enter` to move arm into the original UR5 home position ===============")
        raw_input()
        # appleProxyExperiment.go_to_home()

        apples_to_pick = 10
        for i in range(apples_to_pick):
            apple_proxy_experiment.update_trial_num(i)

            print("\n\n============ Press 'Enter' to move arm into a preliminary starting position ==============")
            raw_input()
            apple_proxy_experiment.go_to_prelim_start()

            # sets up rosbag to subscribe to all topics for each trial
            rosbag_name = apple_proxy_experiment.noise_toString()
            command = "rosbag record -O " + "/home/avl/ur_ws/src/apple_proxy/bag_files/pick" + str(
                i) + rosbag_name + " /success_or_failure /applehand/finger1/imu /applehand/finger1/jointstate /applehand/finger1/pressures /applehand/finger2/imu /applehand/finger2/jointstate /applehand/finger2/pressures /applehand/finger3/imu /applehand/finger3/jointstate /applehand/finger3/pressures wrench joint_states /camera/image_raw /apple_trial_events"
            command = shlex.split(command)
            rosbag_proc = subprocess.Popen(command)

            print("============ Press `Enter` to move arm into the IDEAL starting position and pose =========")
            raw_input()
            apple_proxy_experiment.go_to_starting_position()
            print("Ideal Starting Position reached")

            # makes sure gripper is open first
            print("============ Press `Enter` to open the gripper   ====================================")
            raw_input()
            apple_proxy_experiment.openHandService()

            print("=========== Press 'Enter' to add NOISE to the ideal starting position and pose ===========")
            raw_input()

            apple_proxy_experiment.add_angular_noise()

            print("Noise added to the Ideal Starting Position ")

            print("============ Press `Enter` to go and pick the apple   ====================================")
            raw_input()
            apple_proxy_experiment.go_to_pick_apple()

            print("============ Press `Enter` to close the gripper   ====================================")
            raw_input()
            apple_proxy_experiment.publish_event(0)
            apple_proxy_experiment.closeHandService()

            # stores data about grasp information by describing contact of the proximal and distal links
            csv_data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            csv_data[0] = apple_proxy_experiment.get_pitch()

            print("F0 proximal link(1-contact, 0-no contact) : ")
            f0_proximal = raw_input()
            csv_data[1] = f0_proximal
            print("F0 distal link(1-contact, 0-no contact) : ")
            f0_distal = raw_input()
            csv_data[2] = f0_distal

            print("F1 proximal link(1-contact, 0-no contact) : ")
            f1_proximal = raw_input()
            csv_data[3] = f1_proximal
            print("F1 distal link(1-contact, 0-no contact) : ")
            f1_distal = raw_input()
            csv_data[4] = f1_distal

            print("F2 proximal link(1-contact, 0-no contact) : ")
            f2_proximal = raw_input()
            csv_data[5] = f2_proximal
            print("F2 distal link(1-contact, 0-no contact) : ")
            f2_distal = raw_input()
            csv_data[6] = f2_distal

            print("============ Press 'Enter' to retrieve normally ==========================================")
            raw_input()
            apple_proxy_experiment.publish_event(1)
            apple_proxy_experiment.retrieve()

            time.sleep(2)

            # stops rosbag recording after each trial
            for proc in psutil.process_iter():
                if "record" in proc.name() and set(command[2:]).issubset(proc.cmdline()):
                    proc.send_signal(subprocess.signal.SIGINT)

            rosbag_proc.send_signal(subprocess.signal.SIGINT)

            print("Slip during retrieval(y/n): ")
            slip = str(raw_input())
            csv_data[7] = slip

            print("Drop during retrieval(y/n): ")
            drop = str(raw_input())
            csv_data[8] = drop

            print("Stem shift during retrieval(y/n): ")
            stem_shift = str(raw_input())
            csv_data[9] = stem_shift

            print("==Enter 's' or 'f' for pick success: ")
            answer = str(raw_input())
            apple_proxy_experiment.success_or_failure(answer)
            csv_data[10] = answer

            apple_pose_data = apple_proxy_experiment.get_apple_pose()
            csv_data[11] = apple_pose_data[0]
            csv_data[12] = apple_pose_data[1]
            csv_data[13] = apple_pose_data[2]

            ideal_starting = apple_proxy_experiment.hand_ideal_starting_position_data()
            csv_data[14] = ideal_starting[0]
            csv_data[15] = ideal_starting[1]
            csv_data[16] = ideal_starting[2]
            csv_data[17] = ideal_starting[3]
            csv_data[18] = ideal_starting[4]
            csv_data[19] = ideal_starting[5]

            ideal_picking_pos = apple_proxy_experiment.hand_ideal_picking_position_data()
            csv_data[20] = ideal_picking_pos[0]
            csv_data[21] = ideal_picking_pos[1]
            csv_data[22] = ideal_picking_pos[2]

            actual_picking_pos = apple_proxy_experiment.hand_picking_position_with_noise()
            csv_data[23] = actual_picking_pos[0]
            csv_data[24] = actual_picking_pos[1]
            csv_data[25] = actual_picking_pos[2]

            all_noise = apple_proxy_experiment.get_noise_added()
            csv_data[26] = all_noise[0]
            csv_data[27] = all_noise[1]
            csv_data[28] = all_noise[2]
            csv_data[29] = all_noise[3]
            csv_data[30] = all_noise[4]
            csv_data[31] = all_noise[5]

            apple_proxy_experiment.write_csv(csv_data)

            # TO DO: This is where we want to include the code to open the gripper
            print("============ Press `Enter` to open the gripper   ====================================")
            raw_input()
            apple_proxy_experiment.openHandService()

            # Time in seconds
            time.sleep(1)
            print("Apple No.", i)

            """
            # stops rosbag recording after each trial
            for proc in psutil.process_iter():
                if "record" in proc.name() and set(command[2:]).issubset(proc.cmdline()):
                    proc.send_signal(subprocess.signal.SIGINT)

            rosbag_proc.send_signal(subprocess.signal.SIGINT)
            """

            print("============ Press `Enter` to start next trial  ====================================")
            raw_input()

        print("================================= Apple experiment complete! ===============================")


    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
