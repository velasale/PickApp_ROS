#!/usr/bin/env python
"""
This code is a tool to gather points in the real world, by using the UR5 as a probe,
and with those points calculate the location of the center of the apple and the orientation of the stem
with respect to the arm's base link
"""

# System Related Packages
import sys
import rospy
import geometry_msgs.msg
import csv
# import sympy as sym

# MoveIt Packages
import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list

# Transformation Packages
import tf
import tf2_ros
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_about_axis

# Math Packages
import math
from math import pi


class ScanApplesAndStems(object):

    def __init__(self):
        super(ScanApplesAndStems, self).__init__()

        #....................... PART 1 - INITIAL SETUP ...........................

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('scan_apples_and_stem', anonymous=True)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        #.............PART 2 - DISPLAY BASIC INFORMATION .........................
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")

        #............... PART 3 - GLOBAL VARIABLES AND PARAMETERS ..................
        self.move_group = move_group
        self.probe_length = 0.1  # Length of probe in m
        self.probe_base_width = 1.3 * 0.0254  # Width of the base of the probe in m
        self.ref_frame = "base_link"

    def readPoint(self):

        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        probe_tool = tf2_geometry_msgs.PoseStamped()
        # Enter the coordinates of the probe's tip in the 'tool0' frame
        probe_tool.pose.position.x = 0
        probe_tool.pose.position.y = 0
        probe_tool.pose.position.z = self.probe_length + self.probe_base_width
        probe_tool.header.frame_id = "tool0"
        probe_tool.header.stamp = rospy.Time(0)

        try:
            probe_base = tf_buffer.transform(probe_tool, self.ref_frame, rospy.Duration(2))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        # Only pass the x,y,z coordinates of the pose
        coord = [probe_base.pose.position.x, probe_base.pose.position.y, probe_base.pose.position.z]
        print("The probe coordinates at the 'base_link' reference frame are:", coord)

        return coord


def main():
    try:
        print("----------------------------------------")
        print("-----------Scan apples and stems--------")
        print("--------- Press 'Enter' to begin -------")
        raw_input()
        scan_apples_and_stems = ScanApplesAndStems()

        apples_to_scan = 50
        for i in range(41, apples_to_scan):
            print("--- APPLE ---")
            print("-------------")

            csv_data = [0] * 24
            print("--- Type the apple's diameter in cm (e.g. 8.4): ")
            app_diam = raw_input()
            csv_data[23] = app_diam

            print("--- Place the probe in Point 1 of the apple, and hit Enter when ready")
            raw_input()
            app_pt_1 = scan_apples_and_stems.readPoint()

            print("--- Place the probe in Point 2 of the apple, and hit Enter when ready")
            raw_input()
            app_pt_2 = scan_apples_and_stems.readPoint()

            print("--- Place the probe in Point 3 of the apple, and hit Enter when ready")
            raw_input()
            app_pt_3 = scan_apples_and_stems.readPoint()

            #print("--- Place the probe in Point 4 of the apple, and hit Enter when ready")
            #raw_input()
            #app_pt_4 = scan_apples_and_stems.readPoint()

            # Note: We take a fifth point in order to average the 4 different spheres from the 5 points
            #print("--- Place the probe in Point 5 of the apple, and hit Enter when ready")
            #raw_input()
            #app_pt_5 = scan_apples_and_stems.readPoint()


            # scan_apples_and_stems.appleProperties(p1, p2, p3, p4)

            print("--- STEM ---")
            print("-------------")
            print("--- Place the probe in Point 1 of the STEM (Apple's side), and hit Enter when ready")
            raw_input()
            stm_pt_1 = scan_apples_and_stems.readPoint()

            print("--- Place the probe in Point 2 of the STEM (Branch's side), and hit Enter when ready")
            raw_input()
            stm_pt_2 = scan_apples_and_stems.readPoint()


            csv_data[0] = i
            csv_data[1] = scan_apples_and_stems.ref_frame
            csv_data[2] = app_pt_1[0]
            csv_data[3] = app_pt_1[1]
            csv_data[4] = app_pt_1[2]
            csv_data[5] = app_pt_2[0]
            csv_data[6] = app_pt_2[1]
            csv_data[7] = app_pt_2[2]
            csv_data[8] = app_pt_3[0]
            csv_data[9] = app_pt_3[1]
            csv_data[10] = app_pt_3[2]
            #csv_data[11] = app_pt_4[0]
            #csv_data[12] = app_pt_4[1]
            #csv_data[13] = app_pt_4[2]
            #csv_data[14] = app_pt_5[0]
            #csv_data[15] = app_pt_5[1]
            #csv_data[16] = app_pt_5[2]
            csv_data[17] = stm_pt_1[0]
            csv_data[18] = stm_pt_1[1]
            csv_data[19] = stm_pt_1[2]
            csv_data[20] = stm_pt_2[0]
            csv_data[21] = stm_pt_2[1]
            csv_data[22] = stm_pt_2[2]

            header = ["apple #", "frame_id", "app_p1_x", "app_p1_y", "app_p1_z",
                      "app_p2_x", "app_p2_y", "app_p2_z",
                      "app_p3_x", "app_p3_y", "app_p3_z",
                      "app_p4_x", "app_p4_y", "app_p4_z",
                      "app_p5_x", "app_p5_y", "app_p5_z",
                      "stm_p1_x", "stm_p1_y", "stm_p1_z",
                      "stm_p2_x", "stm_p2_y", "stm_p2_z", "apple's diameter"]
            csv_name = "/home/avl/ur_ws/src/apple_proxy/coords/apple_scan" + str(i) + ".csv"
            with open(csv_name, 'wb') as f:
                writer = csv.writer(f)
                writer.writerow(header)
                writer.writerow(csv_data)


    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
