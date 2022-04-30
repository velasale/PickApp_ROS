#!/usr/bin/env python
"""
Code for picking an apple from the proxy, by placing the end effector (hand) at a position on the surface of a sphere
around the apple.

Alejandro Velasquez
velasale@oregonstate.edu

"""
# --- System related packages
import sys, copy, time, rospy, os, subprocess, shlex, psutil
# --- Math related packages
import math
import numpy as np
from numpy import pi, cos, sin, arccos, arange
from random import random
import sympy as sym
import statistics as st
# --- ROS related packages
import moveit_msgs.msg, geometry_msgs.msg, moveit_commander
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Polygon
from moveit_commander.conversions import pose_to_list
import tf, tf2_ros
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_about_axis, quaternion_from_euler
from std_msgs.msg import String, Int32
from visualization_msgs.msg import Marker, MarkerArray
# --- Database related packages
import csv
# --- Visualization packages
import matplotlib.pyplot as pp


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

        ## .................. PART 1 - INITIAL SETUP .......................

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

        group_name = "manipulator"
        # group_name = "endeffector"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        success_or_failure_publisher = rospy.Publisher('/success_or_failure', String, queue_size=20)
        # this topic is published to at the start of major events in each trial and is recorded to the rosbag(0-beginnig of close hand, 1-beginning of apple retrieval)
        event_publisher = rospy.Publisher('/apple_trial_events', Int32, queue_size=20)

        ## .............PART 2 - DISPLAY BASIC INFORMATION .........................
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

        ## ............... PART 3 - GLOBAL VARIABLES AND PARAMETERS ..................

        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        # self.move_group.set_goal_position_tolerance(0.05)
        # self.move_group.set_goal_orientation_tolerance(0.05)
        self.display_trajectory_publisher = display_trajectory_publisher
        self.success_or_failure_publisher = success_or_failure_publisher
        self.event_publisher = event_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

        # Apple's and Stem Position
        self.apple_pos_x = - 0.497  # Apple position in x-axis [m]
        self.apple_pos_y = - 0.278  # Apple position in y-axis [m]
        self.apple_pos_z = + 1.306  # Apple position in z-axis [m] : Height from the + Distance from table to,
        self.apple_diam = 7.5  # Apple's diameter [cm]
        self.apple_at_baselink = [0, 0, 0]

        self.calix = [-0.499, -0.305, 1.286]  # Calix coordinates
        self.calix_at_baselink = [0, 0, 0]
        self.stem = [-0.495, -0.252, 1.325]  # Point of stem
        self.stem_at_baselink = [0, 0, 0]
        self.stem_socket_at_baselink = [0, 0, 0]
        self.stem_socket = [0, 0, 0]
        self.stem_to_gravity = 0


        # End Effector's psoe
        self.pose_at_baselink = [0, 0, 0, 0, 0, 0, 0]  # x,y,z  and quaternion

        # Vector that points from the initial starting position, towards the center of the apple
        self.vector = [0, 0, 0]
        self.cross_vector = [0, 0, 0]

        # Variables for the markers (sampling sphere, apple, ideal starting points)
        self.marker_id = 1
        self.proxy_markers = MarkerArray()
        self.markerPublisher = rospy.Publisher('balloons', MarkerArray, queue_size=1000)
        self.markerTextPublisher = rospy.Publisher('captions', Marker, queue_size=1000)

        wiper = Marker()
        wiper.id = 0
        wiper.action = wiper.DELETEALL
        self.proxy_markers.markers.append(wiper)
        self.markerPublisher.publish(self.proxy_markers)

        # Arrays to keep track of the Pose achievements
        self.pose_starts = []
        self.pose_yaws = []
        self.pose_noises = []
        self.pose_approaches = []
        self.pose_retrievals = []

        # Offset angle between a finger a the 0 YAW of Lisas's hand, which is required for the yaw adjustment
        self.finger_yaw = - 20 * pi / 180

        self.x_coord = []
        self.y_coord = []
        self.z_coord = []

        # trial number is used later in the program to make sure the csv and the corresponding bag file have the same pick #
        self.trial_number = 0

        # Probe's parameters
        self.probe_length = 0.1  # Length of probe in m
        self.probe_base_width = 1.3 * 0.0254  # Width of the base of the probe in m
        self.ref_frame = "world"

    ## ... Hand Related Functions...

    def close_hand_service(self):
        if self.use_hand:
            os.system("rosservice call /applehand/close_hand")

    def open_hand_service(self):
        if self.use_hand:
            os.system("rosservice call /applehand/open_hand")

    def relax_hand_service(self):
        if self.use_hand:
            os.system("rosservice call /applehand/relax_hand")

    ## ... Rviz marker related functions

    def place_marker_text(self, x, y, z, scale, text):
        """
    Creates a text as a Marker
    @ r,g,b: Indexes of the color in rgb format
    @ a: Alpha value - from 0 (invisible) to 1 (opaque)
    @ x,y,z: coordinates of the marker
    @ scale
    @ text: Text to display in RVIZ
    """
        # Create a marker.  Markers of all shapes share a common type.
        caption = Marker()

        # Set the frame ID and type.  The frame ID is the frame in which the position of the marker
        # is specified.  The type is the shape of the marker, detailed on the wiki page.
        caption.header.frame_id = "/world"
        caption.type = caption.TEXT_VIEW_FACING

        # Each marker has a unique ID number.  If you have more than one marker that you want displayed at a
        # given time, then each needs to have a unique ID number.  If you publish a new marker with the same
        # ID number and an existing marker, it will replace the existing marker with that ID number.
        caption.id = 0

        # Set the action.  We can add, delete, or modify markers.
        caption.action = caption.ADD

        # These are the size parameters for the marker.  The effect of these on the marker will vary by shape,
        # but, basically, they specify how big the marker along each of the axes of the coordinate frame named
        # in frame_id.
        caption.scale.x = scale
        caption.scale.y = scale
        caption.scale.z = scale

        # Color, as an RGB triple, from 0 to 1.
        caption.color.r = 1
        caption.color.g = 1
        caption.color.b = 1
        caption.color.a = 1

        caption.text = text

        # Specify the pose of the marker.  Since spheres are rotationally invarient, we're only going to specify
        # the positional elements.  As usual, these are in the coordinate frame named in frame_id.  Every time the
        # marker is displayed in rviz, ROS will use tf to determine where the marker should appear in the scene.
        # in this case, the position will always be directly above the robot, and will move with it.
        caption.pose.position.x = x
        caption.pose.position.y = y
        caption.pose.position.z = z

        # Set up a publisher.  We're going to publish on a topic called balloon.
        self.markerTextPublisher.publish(caption)

        # Set a rate.  10 Hz is a good default rate for a marker moving with the Fetch robot.
        rate = rospy.Rate(10)

    def place_marker_sphere(self, r, g, b, a, x, y, z, scale):
        """
    Creates a Sphere as a marker, and appends it into an array of Markers
    @ r,g,b: Indexes of the color in rgb format
    @ a: Alpha value - from 0 (invisible) to 1 (opaque)
    @ x,y,z: coordinates of the marker
    """
        # Create a marker.  Markers of all shapes share a common type.
        sphere = Marker()

        # Set the frame ID and type.  The frame ID is the frame in which the position of the marker
        # is specified.  The type is the shape of the marker, detailed on the wiki page.
        sphere.header.frame_id = "/world"
        sphere.type = sphere.SPHERE

        # Each marker has a unique ID number.  If you have more than one marker that you want displayed at a
        # given time, then each needs to have a unique ID number.  If you publish a new marker with the same
        # ID number and an existing marker, it will replace the existing marker with that ID number.
        sphere.id = self.marker_id + 1
        self.marker_id = sphere.id

        # Set the action.  We can add, delete, or modify markers.
        sphere.action = sphere.ADD

        # These are the size parameters for the marker.  The effect of these on the marker will vary by shape,
        # but, basically, they specify how big the marker along each of the axes of the coordinate frame named
        # in frame_id.
        sphere.scale.x = scale
        sphere.scale.y = scale
        sphere.scale.z = scale

        # Color, as an RGB triple, from 0 to 1.
        sphere.color.r = r
        sphere.color.g = g
        sphere.color.b = b
        sphere.color.a = a

        # Specify the pose of the marker.  Since spheres are rotationally invarient, we're only going to specify
        # the positional elements.  As usual, these are in the coordinate frame named in frame_id.  Every time the
        # marker is displayed in rviz, ROS will use tf to determine where the marker should appear in the scene.
        # in this case, the position will always be directly above the robot, and will move with it.
        sphere.pose.position.x = x
        sphere.pose.position.y = y
        sphere.pose.position.z = z
        sphere.pose.orientation.x = 0.0
        sphere.pose.orientation.y = 0.0
        sphere.pose.orientation.z = 0.0
        sphere.pose.orientation.w = 1.0

        self.proxy_markers.markers.append(sphere)
        # Set up a publisher.  We're going to publish on a topic called balloon.
        self.markerPublisher.publish(self.proxy_markers)

        # Set a rate.  10 Hz is a good default rate for a marker moving with the Fetch robot.
        rate = rospy.Rate(10)

    def place_marker_arrow(self, p1, p2):

        stem = Marker()
        stem.header.frame_id = "world"
        stem.type = stem.ARROW

        # Since we want to publish an array of markers, the id must be updated.
        stem.id = self.marker_id + 1
        self.marker_id = stem.id

        stem.action = stem.ADD

        # For line markers, only sclae.x works, and it defines the line width
        stem.scale.x = 0.01  # shaft diameter
        stem.scale.y = 0.02  # head diameter
        stem.scale.z = 0.02  # head length

        stem.color.r = 0
        stem.color.g = 1
        stem.color.b = 0
        stem.color.a = 1

        # Translate points into floats
        p1x = float(p1[0])
        p1y = float(p1[1])
        p1z = float(p1[2])
        p2x = float(p2[0])
        p2y = float(p2[1])
        p2z = float(p2[2])

        p1 = Point()
        p1.x = p1x
        p1.y = p1y
        p1.z = p1z

        p2 = Point()
        p2.x = p2x
        p2.y = p2y
        p2.z = p2z

        stem.points.append(p1)
        stem.points.append(p2)

        self.proxy_markers.markers.append(stem)
        self.markerPublisher.publish(self.proxy_markers)

        rate = rospy.Rate(10)

    def place_apple_and_stem(self):
        # Place marker to see the apple in Rviz
        self.place_marker_sphere(1, 0, 0, 0.5, self.apple_pos_x, self.apple_pos_y, self.apple_pos_z,
                                 self.apple_diam / 100)
        # Place marker to see the Stem in Rviz
        self.place_marker_arrow(self.calix, self.stem)

    ## ... Probe related functions

    def scan_apple_and_stem(self):

        # Step 1 - Scan the points
        # print("--- Type the apple's diameter in cm (e.g. 8.4): ")
        # app_diam = float(raw_input())
        # self.apple_diam = app_diam

        self.apple_diam = 7.5       # Default diameter of the apple used at the proxy

        # print("--- Scanning Apple")
        # print("--- Place the probe in Point 1 of the apple, and hit Enter when ready")
        # raw_input()
        # app_pt_1 = self.read_point()
        #
        # print("--- Place the probe in Point 2 of the apple, and hit Enter when ready")
        # raw_input()
        # app_pt_2 = self.read_point()
        #
        # print("--- Place the probe in Point 3 of the apple, and hit Enter when ready")
        # raw_input()
        # app_pt_3 = self.read_point()

        # print("The apple points are: ")
        # print(app_pt_1)
        # print(app_pt_2)
        # print(app_pt_3)

        print("--- Scanning Stem")
        print("--- Place the probe in Point 1 of the STEM (Calix), and hit Enter when ready")
        raw_input()
        stm_pt_1 = self.read_point()
        self.calix = stm_pt_1

        print("--- Place the probe in Point 2 of the STEM (Branch's side), and hit Enter when ready")
        raw_input()
        stm_pt_2 = self.read_point()
        self.stem = stm_pt_2

        print("--- Place the probe in Point 3 of the STEM (Socket's side), and hit Enter when ready")
        raw_input()
        stm_pt_3 = self.read_point()
        self.stem_socket = stm_pt_3


        print("The stem points are: ")
        print(stm_pt_1)
        print(stm_pt_2)
        print(stm_pt_3)

        delta_z = self.stem[2] - self.calix[2]
        delta_y = self.stem[1] - self.calix[1]
        self.stem_to_gravity = 90 + math.degrees(math.atan(delta_z / delta_y))
        print("The Stem-Gravity angle is: ", self.stem_to_gravity)

        # Step 2 - Estimate center's location
        # a1, b1, c1 = self.estimate_apple_center(app_pt_1, app_pt_2, app_pt_3, app_diam)
        a2, b2, c2 = self.center_from_stem(stm_pt_1, stm_pt_2)

        # Finally average them to have a better result
        # a = (a1 + a2) / 2
        # b = (b1 + b2) / 2
        # c = (c1 + c2) / 2
        a = a2
        b = b2
        c = c2

        self.apple_pos_x = float(a)
        self.apple_pos_y = float(b)
        self.apple_pos_z = float(c)

        print("The center is located at: ", round(a, 3), round(b, 3), round(c, 3))

    def estimate_apple_center(self, app_p1, app_p2, app_p3, diameter):
        """
    Obtain the center of a sphere, given 3 points and its diameter
    :param app_p1:
    :param app_p2:
    :param app_p3:
    :param diameter: Apple's diameter given in [m]
    :return:
    """
        # --- Step 1: Define the variables of the Sphere
        a, b, c = sym.symbols('a, b, c')  # Coordinates of the center of the sphere / apple
        r = sym.Symbol('r')  # Radius of the sphere / apple

        p = 2
        f = 1

        # Warning: sympy doesn't like arrays, nor more decimals than 2
        p1 = [round(app_p1[0] * f, p), round(app_p1[1] * f, p), round(app_p1[2] * f, p)]  # round
        p2 = [round(app_p2[0] * f, p), round(app_p2[1] * f, p), round(app_p2[2] * f, p)]
        p3 = [round(app_p3[0] * f, p), round(app_p3[1] * f, p), round(app_p3[2] * f, p)]

        radius = float(diameter) / 200  # In [m]
        # radius = diameter / 2  # In [cm]

        try:
            print("calculating the center...")
            # --- Step 3: Elaborate the system of equations, in this case the equation of a sphere
            eq1 = sym.Eq((p1[0] - a) ** 2 + (p1[1] - b) ** 2 + (p1[2] - c) ** 2, radius ** 2)
            eq2 = sym.Eq((p2[0] - a) ** 2 + (p2[1] - b) ** 2 + (p2[2] - c) ** 2, radius ** 2)
            eq3 = sym.Eq((p3[0] - a) ** 2 + (p3[1] - b) ** 2 + (p3[2] - c) ** 2, radius ** 2)
            sln_1 = sym.solve([eq1, eq2, eq3], (a, b, c), warn=True)

            # print("Given Apple's diameter: %.2f cm" % diameter)
            # print("Average of Apple's center (x, y, z): %.2f %.2f %.2f " % (sln_1[1][0], sln_1[1][1], sln_1[1][2]))

            return sln_1[1][0], sln_1[1][1], sln_1[1][2]


        except TypeError or KeyError or ValueError:
            print("exception")
            # Adjust a bit the radius, so it finds a solution
            radius = 1.01 * radius  # In [m]

            # --- Step 3: Elaborate the system of equations, in this case the equation of a sphere
            eq1 = sym.Eq((p1[0] - a) ** 2 + (p1[1] - b) ** 2 + (p1[2] - c) ** 2, radius ** 2)
            eq2 = sym.Eq((p2[0] - a) ** 2 + (p2[1] - b) ** 2 + (p2[2] - c) ** 2, radius ** 2)
            eq3 = sym.Eq((p3[0] - a) ** 2 + (p3[1] - b) ** 2 + (p3[2] - c) ** 2, radius ** 2)
            sln_1 = sym.solve([eq1, eq2, eq3], (a, b, c))

            print(".... trying a bit bigger radius")
            print("Given Apple's diameter: %.2f cm" % (100 * 2 * radius))
            print("Average of Apple's center (x, y, z): %.2f %.2f %.2f " % (sln_1[1][0], sln_1[1][1], sln_1[1][2]))

            return sln_1[1][0], sln_1[1][1], sln_1[1][2]

    def center_from_stem(self, stm_p1, stm_p2):

        a = (stm_p1[0] + stm_p2[0]) / 2
        b = (stm_p1[1] + stm_p2[1]) / 2
        c = (stm_p1[2] + stm_p2[2]) / 2

        return a, b, c

    def read_point(self):

        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        probe_tool = tf2_geometry_msgs.PoseStamped()
        # Enter the coordinates of the probe's tip in the 'tool0' frame
        probe_tool.pose.position.x = 0
        probe_tool.pose.position.y = 0
        # Take the distance from the tip to the reference frame
        probe_tool.pose.position.z = self.probe_length + self.probe_base_width - 0.165  # WARNING - look ur5e_robot.urdf.xacro
        probe_tool.header.frame_id = "palm"
        probe_tool.header.stamp = rospy.Time(0)

        try:
            probe_base = tf_buffer.transform(probe_tool, self.ref_frame, rospy.Duration(2))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        # Only pass the x,y,z coordinates of the pose
        coord = [float(probe_base.pose.position.x), float(probe_base.pose.position.y),
                 float(probe_base.pose.position.z)]
        # print("The probe coordinates at the 'base_link' reference frame are:", coord)

        return coord

    ## ... Arm related functions

    def go_home(self):
        # --- Initiate object joint goal
        joint_goal = self.move_group.get_current_joint_values()

        # --- Initial default joint values
        joint_goal[0] = - pi / 2
        joint_goal[1] = 0
        joint_goal[2] = - pi * 145 / 180
        joint_goal[3] = - pi * 3 / 4
        joint_goal[4] = - pi / 2
        joint_goal[5] = 0

        # --- Move to the joint goal
        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()

        # --- Compare the joint goal with the current joint state
        current_joints = self.move_group.get_current_joint_values()
        # Print for debugging:
        # print("Final Joints State: ", current_joints)
        return all_close(joint_goal, current_joints, 0.01)

    def go_preliminary_position(self):
        """ This function is to avoid the robot from travelling around weird points"""

        # # Place a marker for the apple
        # self.place_marker_sphere(1, 0, 0, 1.0, self.apple_pos_x, self.apple_pos_y, self.apple_pos_z, 0.08)
        # # Place a marker for the sampling sphere
        # self.place_marker_sphere(0, 1, 0, 0.2, self.apple_pos_x, self.apple_pos_y, self.apple_pos_z, self.sphereRadius * 2)

        # --- Place a marker for the text
        self.place_marker_text(self.apple_pos_x, self.apple_pos_y, self.apple_pos_z + 0.5, 0.1,
                               "Going to Preliminary Starting Position")

        # --- Initiate object joint goal
        joint_goal = self.move_group.get_current_joint_values()

        # --- Preliminary position joint values
        joint_goal[0] = -   0 * pi / 180
        joint_goal[1] = - 100 * pi / 180
        joint_goal[2] = - 125 * pi / 180
        joint_goal[3] = - 100 * pi / 180
        joint_goal[4] = -  90 * pi / 180
        joint_goal[5] = 0

        # --- Move to the joint goal
        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()

        # --- Compare the joint goal with the current joint state
        current_joints = self.move_group.get_current_joint_values()
        # Print for debugging:
        # print("Final Joints State: ", current_joints)

        return all_close(joint_goal, current_joints, 0.01)

    def go_to_starting_position(self, index):
        """ This function takes the gripper to the IDEAL starting position, before adding noise.
        * Places the gripper in a position on the surface of the sphere (center at the apple's)
        * Orients the gripper so that its Z-axis points towards the center of the apple.
    """

        # --- Place a marker with text in RVIZ
        text = "Going to an IDEAL Starting Position # " + str(index)
        self.place_marker_text(self.apple_pos_x, self.apple_pos_y, self.apple_pos_z + 0.5, 0.1, text)

        # --- Step 1: Find Gripper's Goal Pose Position
        # Center of Sphere = Location of apple
        h = self.apple_pos_x
        k = self.apple_pos_y
        l = self.apple_pos_z

        # Coordinates of the point sampled on the sphere's surface
        x = self.x_coord[index] + h
        y = self.y_coord[index] + k
        z = self.z_coord[index] + l

        # --- Step 2: Find Gripper's Goal Pose Orientation
        self.azimuth = math.atan2(self.y_coord[index], self.x_coord[index]) * 180 / pi
        # Get the vector pointing to the center of the sphere
        delta_x = x - h
        delta_y = y - k
        delta_z = z - l
        length = math.sqrt(delta_x ** 2 + delta_y ** 2 + delta_z ** 2)

        # Cross Product between that vector V2 and the vector normal to the palm "z" V1 to obtain the rotation vector V3
        V1 = [0, 0, 1]
        V2 = [-delta_x / length, -delta_y / length, -delta_z / length]  # Normalize it
        V3 = np.cross(V1, V2)

        self.vector = [delta_x, delta_y, delta_z]
        self.cross_vector = V3

        # Dot Product to obtain the angle of rotation
        dot = np.dot(V1, V2)
        angle = math.acos(dot)

        # Obtain the quaternion from the single axis rotation
        q = quaternion_about_axis(angle, V3)
        # Also have them as RPY
        roll, pitch, yaw = euler_from_quaternion(q)
        self.initial_pitch_angle = pitch / pi * 180
        self.initial_yaw_angle = yaw / pi * 180
        self.initial_roll_angle = roll / pi * 180

        # Initiate pose object
        pose_goal = self.move_group.get_current_pose().pose
        # Assign Orientation values
        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2]
        pose_goal.orientation.w = q[3]
        # Assign Position values
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        # Save the ideal pose position in globals
        self.ideal_starting_x = pose_goal.position.x
        self.ideal_starting_y = pose_goal.position.y
        self.ideal_starting_z = pose_goal.position.z

        # --- Step 3: Move to the goal pose
        self.move_group.set_pose_target(pose_goal)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()

        # --- Compare the joint goal with the current joint state
        current_pose = self.move_group.get_current_pose().pose

        # Save this pose in a global variable, to come back to it after picking the apple
        self.previous_pose = current_pose

        # Place a blue dot in the sphere's surface to keep track of all the sampled points
        self.place_marker(0, 0, 1, 1, x, y, z, 0.02)

        success = all_close(pose_goal, current_pose, 0.01)
        self.pose_starts.append(success)
        print("Pose Starts history", self.pose_starts)

        return success

    def align_with_stem(self):
        """This function takes the gripper to the IDEAL starting position, before adding noise

    """

        text = "Going to an IDEAL Starting Position"
        # Place a marker for the text
        self.place_marker_text(self.apple_pos_x, self.apple_pos_y, self.apple_pos_z + 0.5, 0.1,
                               text)

        # Listen to the TF topic
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        # --- Step 1: Assign Gripper's Goal Pose Position
        # Read current pose
        current_pose = self.move_group.get_current_pose().pose
        # Translate the apple's it into the tool's c-frame
        pose_at_world = tf2_geometry_msgs.PoseStamped()
        pose_at_world.pose = current_pose

        # Find the orientation
        delta_x = self.stem[0] - self.calix[0]
        delta_y = self.stem[1] - self.calix[1]
        delta_z = self.stem[2] - self.calix[2]
        roll = math.atan2(-delta_y, delta_z)
        pitch = math.radians(0)
        yaw = math.radians(0)
        q = quaternion_from_euler(roll, pitch, yaw)

        pose_at_world.pose.orientation.x = q[0]
        pose_at_world.pose.orientation.y = q[1]
        pose_at_world.pose.orientation.z = q[2]
        pose_at_world.pose.orientation.w = q[3]

        # 1st Rotate
        self.move_group.set_pose_target(pose_at_world.pose)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        # 2nd Translate
        pose_at_world.pose.position.x = self.apple_pos_x
        pose_at_world.pose.position.y = self.apple_pos_y
        pose_at_world.pose.position.z = self.apple_pos_z

        pose_at_world.header.frame_id = self.ref_frame
        pose_at_world.header.stamp = rospy.Time(0)

        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            pose_at_tool = tf_buffer.transform(pose_at_world, "palm", rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        pose_at_tool.pose.position.z = pose_at_tool.pose.position.z - 0.04

        # --- Step 3: Convert pose back into reference frame: World's c-frame
        pose_at_tool.header.stamp = rospy.Time(0)

        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            new_pose_at_world = tf_buffer.transform(pose_at_tool, self.ref_frame, rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        # --- Step 4: Finally, move to the new pose
        self.move_group.set_pose_target(new_pose_at_world.pose)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        # Compare poses
        pose_goal = new_pose_at_world.pose
        current_pose = self.move_group.get_current_pose().pose

        # Save pose as an ideal one
        self.ideal_pose = self.move_group.get_current_pose().pose

        print(current_pose)
        success = all_close(pose_goal, current_pose, 0.01)

        return success

    def adjust_yaw(self):
        """
    This function adjusts the YAW of the end tool, so the hand fingers avoid jamming into the STEM.
    Step 1: Transform the stem's vector from the world into the tool's local frame
    Step 2: Cross product the stem's vector in the new frame, with the z axis, to obtain the vector normal to the plane
            created by these two vectors.
    Step 3: Dot product between the rotating vector and the x-axis, subtract pi/2, and obtain the angle to rotate YAW
    Step 4: Rotate YAW and transform it into the world frame
    """

        # --- Place a marker with text in RVIZ
        self.place_marker_text(self.apple_pos_x, self.apple_pos_y, self.apple_pos_z + 0.5, 0.1,
                               "Adjust YAW... so the fingers encage the apple")

        # --- Step 1: Read the pose from the "Base_link" into "Tool0"
        # **Assuming /tf2 topic is being broadcasted
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        # Point A of Stem
        pose_stamped_A = tf2_geometry_msgs.PoseStamped()
        pose_stamped_A.pose.position.x = 0
        pose_stamped_A.pose.position.y = 0
        pose_stamped_A.pose.position.z = 0

        pose_stamped_A.header.frame_id = "base_link"
        pose_stamped_A.header.stamp = rospy.Time(0)

        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            output_pose_stamped_A = tf_buffer.transform(pose_stamped_A, "tool0", rospy.Duration(1))

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        # Point B of Stem - In this case the Stem is Aligned with Z-Axis
        pose_stamped_B = tf2_geometry_msgs.PoseStamped()
        pose_stamped_B.pose.position.x = 0
        pose_stamped_B.pose.position.y = 0
        pose_stamped_B.pose.position.z = 1

        pose_stamped_B.header.frame_id = "base_link"
        pose_stamped_B.header.stamp = rospy.Time(0)

        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            output_pose_stamped_B = tf_buffer.transform(pose_stamped_B, "tool0", rospy.Duration(1))
            # print("Point B in Tool0: ", output_pose_stamped_B.pose)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        # Points A and B transformed into the tool c-frame
        C = [output_pose_stamped_A.pose.position.x, output_pose_stamped_A.pose.position.y,
             output_pose_stamped_A.pose.position.z]
        D = [output_pose_stamped_B.pose.position.x, output_pose_stamped_B.pose.position.y,
             output_pose_stamped_B.pose.position.z]

        # And E is the final vector that represents the Stem in the tool c-frame
        E = np.subtract(D, C)
        # Normalize vector
        length_E = math.sqrt(E[0] ** 2 + E[1] ** 2 + E[2] ** 2)
        E[0] = E[0] / length_E
        E[1] = E[1] / length_E
        E[2] = E[2] / length_E

        # --- Step 2: Cross product between vector E and tools'z Z vector
        Z = [0, 0, 1]
        F = np.cross(E, Z)
        # Normalize vector
        length_F = math.sqrt(F[0] ** 2 + F[1] ** 2 + F[2] ** 2)
        F[0] = F[0] / length_F
        F[1] = F[1] / length_F
        F[2] = F[2] / length_F

        self.elevation = 180 - math.acos(np.dot(E, Z)) * 180 / pi
        print('\n The elevation angle is (angle between stem and local z) and azimuth are', self.elevation,
              self.azimuth)

        dot = np.dot([1, 0, 0], F)
        angle = math.acos(dot) - pi / 2

        ##################################
        ##################################
        current_pose = self.move_group.get_current_pose().pose

        # --- Step 1: Read the pose from the "Base_link" into "Tool0"
        # **Assuming /tf2 topic is being broadcasted
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        pose_stamped = tf2_geometry_msgs.PoseStamped()
        pose_stamped.pose = current_pose
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.header.stamp = rospy.Time(0)

        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            output_pose_stamped = tf_buffer.transform(pose_stamped, "tool0", rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        second_pose = output_pose_stamped.pose
        quaternion = [second_pose.orientation.x,
                      second_pose.orientation.y,
                      second_pose.orientation.z,
                      second_pose.orientation.w]

        roll, pitch, yaw = euler_from_quaternion(quaternion)

        yaw = yaw + angle + self.finger_yaw
        new_quat = quaternion_from_euler(roll, pitch, yaw)

        second_pose.orientation.x = new_quat[0]
        second_pose.orientation.y = new_quat[1]
        second_pose.orientation.z = new_quat[2]
        second_pose.orientation.w = new_quat[3]

        # --- Step 3: Convert the desired pose back into the "Base_Link" reference frame
        sec_pose_stamped = tf2_geometry_msgs.PoseStamped()
        sec_pose_stamped.pose = second_pose
        sec_pose_stamped.header.frame_id = "tool0"
        sec_pose_stamped.header.stamp = rospy.Time(0)

        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            output_pose_stamped = tf_buffer.transform(sec_pose_stamped, "base_link", rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        # --- Step 4: Finally move to the new position
        self.move_group.set_pose_target(output_pose_stamped.pose)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        # Compare poses
        pose_goal = output_pose_stamped.pose
        current_pose = self.move_group.get_current_pose().pose

        success = all_close(pose_goal, current_pose, 0.01)
        self.pose_yaws.append(success)
        print("Pose Yaws history", self.pose_yaws)

        return success

    def go_pick_apple(self, angular_noise_deg):

        # Place a text marker in Rviz
        self.place_marker_text(self.apple_pos_x, self.apple_pos_y, self.apple_pos_z + 0.5, 0.1,
                               "Approaching apple, and adding ANGULAR noise")

        current_pose = self.move_group.get_current_pose().pose

        # --- Step 1: Read the pose from the "Base_link" into "Tool0"
        # **Assuming /tf2 topic is being broadcasted
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        pose_stamped = tf2_geometry_msgs.PoseStamped()
        pose_stamped.pose = current_pose
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.header.stamp = rospy.Time(0)

        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            output_pose_stamped = tf_buffer.transform(pose_stamped, "tool0", rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        # --- Step 2: Add noise
        # Now that the pose is in the tool's frame, we can add noise easily
        angular_noise = angular_noise_deg * pi / 180
        second_pose = output_pose_stamped.pose
        # second_pose.position.z = second_pose.position.z + self.delta_z
        second_pose.position.z = second_pose.position.z + 0.06

        quaternion = [second_pose.orientation.x,
                      second_pose.orientation.y,
                      second_pose.orientation.z,
                      second_pose.orientation.w]

        roll, pitch, yaw = euler_from_quaternion(quaternion)

        # roll = roll + 1 * (random() * angular_noise - angular_noise / 2)
        # pitch = pitch + 1 * (random() * angular_noise - angular_noise / 2)
        # yaw = yaw + 1 * (random() * angular_noise - angular_noise / 2)

        # Add noise to the ROLL Angle
        roll_noise_rad = self.roll_noise_deg * pi / 180
        # Add noise to the PITCH Angle
        pitch_noise_rad = self.pitch_noise_deg * pi / 180
        # Add noise to the YAW Angle
        yaw_noise_rad = self.yaw_noise_deg * pi / 180

        roll = roll + 1 * (roll_noise_rad)
        pitch = pitch + 1 * (pitch_noise_rad)
        yaw = yaw + 1 * (yaw_noise_rad)

        new_quat = quaternion_from_euler(roll, pitch, yaw)
        second_pose.orientation.x = new_quat[0]
        second_pose.orientation.y = new_quat[1]
        second_pose.orientation.z = new_quat[2]
        second_pose.orientation.w = new_quat[3]

        # --- Step 3: Convert the pose back into the "Base_Link" reference frame
        sec_pose_stamped = tf2_geometry_msgs.PoseStamped()
        sec_pose_stamped.pose = second_pose
        sec_pose_stamped.header.frame_id = "tool0"
        sec_pose_stamped.header.stamp = rospy.Time(0)

        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            output_pose_stamped = tf_buffer.transform(sec_pose_stamped, "base_link", rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        # --- Step 4: Finally move to the new position
        self.move_group.set_pose_target(output_pose_stamped.pose)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        # Compare poses
        pose_goal = output_pose_stamped.pose
        current_pose = self.move_group.get_current_pose().pose

        success = all_close(pose_goal, current_pose, 0.01)
        self.pose_approaches.append(success)
        print("Pose Approaches history", self.pose_approaches)

        return success

    def retrieve(self, distance):

        # Place a marker for the text
        self.place_marker_text(self.apple_pos_x, self.apple_pos_y, self.apple_pos_z + 0.5, 0.1,
                               "Retrieving normally w.r.t. the palm")

        current_pose = self.move_group.get_current_pose().pose

        # --- Step 1: Read the pose from the "Base_link" into "Tool0"
        # **Assuming /tf2 topic is being broadcasted
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        pose_stamped = tf2_geometry_msgs.PoseStamped()
        pose_stamped.pose = current_pose
        pose_stamped.header.frame_id = self.ref_frame
        pose_stamped.header.stamp = rospy.Time(0)

        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            pose_at_tool = tf_buffer.transform(pose_stamped, "palm", rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        pose_at_tool.pose.position.z = pose_at_tool.pose.position.z - distance
        # --- Step 3: Convert the pose back into the "Base_Link" reference frame
        pose_at_tool.header.stamp = rospy.Time(0)

        try:
            # ** It is important to wait for the listener to start listening. Hence, the rospy.Duration(1)
            new_pose_at_world = tf_buffer.transform(pose_at_tool, self.ref_frame, rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        # --- Step 4: Finally, move to the new position
        self.move_group.set_pose_target(new_pose_at_world.pose)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        # Compare poses
        pose_goal = new_pose_at_world.pose
        current_pose = self.move_group.get_current_pose().pose

        success = all_close(pose_goal, current_pose, 0.01)
        self.pose_retrievals.append(success)
        # print("Pose Retrievals history", self.pose_retrievals)

        return success

    ## ... Proxy related functions

    def point_sampling(self):
        """
    This function samples points evenly distributed from the surface of a sphere
    Source: https://stackoverflow.com/questions/9600801/evenly-distributing-n-points-on-a-sphere
    """
        num_pts = 8 * 30
        indices = arange(0, num_pts, dtype=float) + 0.5

        phi = arccos(1 - 2 * indices / num_pts)
        theta = pi * (1 + 5 ** 0.5) * indices

        x, y, z = cos(theta) * sin(phi), sin(theta) * sin(phi), cos(phi);
        # Adjustement due to the distance between World and Base_Link frame

        # Further selection of points for only one quarter of the sphere
        for i in range(len(x)):
            if x[i] < 0:  # To get only half sphere
                if z[i] > 0:  # To get only one quarter of the sphere
                    if y[i] > 0:  # To get only one eight of the sphere
                        self.x_coord.append(x[i] * self.sphereRadius)
                        self.y_coord.append(y[i] * self.sphereRadius)
                        self.z_coord.append(z[i] * self.sphereRadius)

    def add_cartesian_noise(self, x_noise, y_noise, z_noise):

        # --- Place a marker with text in RVIZ
        self.place_marker_text(self.apple_pos_x, self.apple_pos_y, self.apple_pos_z + 0.5, 0.1,
                               "Add CARTESIAN noise in the Tool's local frame")

        # --- Step 1: Read the pose from the "Base_link" into "Tool0"
        # Listen to the tf topic
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)
        # Initiate pose object
        pose_at_world = tf2_geometry_msgs.PoseStamped()
        pose_at_world.pose = self.ideal_pose
        pose_at_world.header.frame_id = self.ref_frame
        pose_at_world.header.stamp = rospy.Time(0)

        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            pose_at_tool = tf_buffer.transform(pose_at_world, "palm", rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        # --- Step 2: Add noise
        # Now that the pose is in the tool's frame, we can add noise easily in the tool's cframe
        pose_at_tool.pose.position.x = pose_at_tool.pose.position.x + x_noise
        pose_at_tool.pose.position.y = pose_at_tool.pose.position.y + y_noise
        pose_at_tool.pose.position.z = pose_at_tool.pose.position.z + z_noise

        # --- Step 3: Convert the pose back into the "Base_Link" reference frame
        pose_at_tool.header.stamp = rospy.Time(0)
        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            new_pose_at_world = tf_buffer.transform(pose_at_tool, self.ref_frame, rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        # --- Step 4: Finally, move to the new pose with noise
        self.move_group.set_pose_target(new_pose_at_world.pose)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        # Compare poses
        pose_goal = new_pose_at_world.pose
        current_pose = self.move_group.get_current_pose().pose
        success = all_close(pose_goal, current_pose, 0.01)

        self.pose_noises.append(success)
        # print("Pose Noises history", self.pose_noises)

        return success

    def add_angular_noise(self, roll_noise, pitch_noise, yaw_noise):

        # --- Place a marker with text in RVIZ
        self.place_marker_text(self.apple_pos_x, self.apple_pos_y, self.apple_pos_z + 0.5, 0.1,
                               "Add ANGULAR noise in the palms's local frame")

        # --- Step 1: Read the current pose from the "Base_link" and convert it into "Tool0"
        # Listen to the tf topic
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)
        # Initiate pose object
        pose_stamped = tf2_geometry_msgs.PoseStamped()

        # If angular noise is added BEFORE the cartesian noise:
        # pose_stamped.pose = self.ideal_pose
        # If angular noise is added AFTER the cartesian noise:
        pose_stamped.pose = self.move_group.get_current_pose().pose

        pose_stamped.header.frame_id = self.ref_frame
        pose_stamped.header.stamp = rospy.Time(0)

        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            output_pose_stamped = tf_buffer.transform(pose_stamped, "palm", rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        # --- Step 2: Add noise
        # Now that the pose is in the tool's frame, we can add noise easily in the tool's cframe
        second_pose = output_pose_stamped.pose

        quaternion = [second_pose.orientation.x,
                      second_pose.orientation.y,
                      second_pose.orientation.z,
                      second_pose.orientation.w]

        roll, pitch, yaw = euler_from_quaternion(quaternion)
        roll = roll + roll_noise
        pitch = pitch + pitch_noise
        yaw = yaw + yaw_noise

        q = quaternion_from_euler(roll, pitch, yaw)

        second_pose.orientation.x = q[0]
        second_pose.orientation.y = q[1]
        second_pose.orientation.z = q[2]
        second_pose.orientation.w = q[3]

        # --- Step 3: Convert the pose back into the "Base_Link" reference frame
        sec_pose_stamped = tf2_geometry_msgs.PoseStamped()
        sec_pose_stamped.pose = second_pose
        sec_pose_stamped.header.frame_id = "palm"
        sec_pose_stamped.header.stamp = rospy.Time(0)

        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            output_pose_stamped = tf_buffer.transform(sec_pose_stamped, self.ref_frame, rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        # --- Step 4: Finally, move to the new pose with noise
        self.move_group.set_pose_target(output_pose_stamped.pose)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        # Compare poses
        pose_goal = output_pose_stamped.pose
        current_pose = self.move_group.get_current_pose().pose
        success = all_close(pose_goal, current_pose, 0.01)

        self.pose_noises.append(success)

        # --- Save pose w.r.t. baselink
        try:
            pose_at_baselink = tf_buffer.transform(output_pose_stamped, "base_link", rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        self.pose_at_baselink[0] = pose_at_baselink.pose.position.x
        self.pose_at_baselink[1] = pose_at_baselink.pose.position.y
        self.pose_at_baselink[2] = pose_at_baselink.pose.position.z
        self.pose_at_baselink[3] = pose_at_baselink.pose.orientation.x
        self.pose_at_baselink[4] = pose_at_baselink.pose.orientation.y
        self.pose_at_baselink[5] = pose_at_baselink.pose.orientation.z
        self.pose_at_baselink[6] = pose_at_baselink.pose.orientation.w

        # print("Pose Noises history", self.pose_noises)

        return success

    def cart_noise(self):
        axes = ['x', 'y', 'z']
        # axes = ['y', 'z']
        # axes = ['x']

        results = []
        for axis in axes:
            for noise in range(0, 10):
                # Add the noise

                if axis == 'x':
                    step = 0.01  # 1cm
                    final_noise = (noise - 10 / 2) * step
                    self.add_cartesian_noise(final_noise, 0, 0)
                elif axis == 'y':
                    step = 0.01  # 1cm
                    final_noise = (noise - 10 / 2) * step
                    self.add_cartesian_noise(0, final_noise, 0)
                else:
                    step = 0.005  # 1cm
                    final_noise = (noise - 10 / 2) * step
                    self.add_cartesian_noise(0, 0, final_noise)

                for pick in range(1):
                    print("Doing pick %i / 10, at %.3f, in %s axis" % (pick, final_noise, axis))
                    dist = 0.1
                    # self.open_hand_service()
                    # raw_input()
                    # self.close_hand_service()

                    # raw_input()
                    self.retrieve(dist)  # Distance in cm
                    # Type the result
                    # print("Enter 's' or 'f' for pick success: ")
                    # answer = ''
                    # while ((answer is not 's') and (answer is not 'f')):
                    #   answer = str(raw_input())
                    # result = [axis, final_noise, answer]
                    # results.append(result)
                    # self.open_hand_service()

                    # raw_input()
                    self.retrieve(-dist)  # Distance in cm

                    # Save the results every time to avoid losing them if anything happens
                    with open('lin_noise_max_results', 'w') as f:
                        write = csv.writer(f)
                        write.writerows(results)

    def ang_noise(self):
        # axes = ['roll', 'pitch']
        axes = ['pitch']
        results = []
        for axis in axes:
            # Sweep all the axes

            noise_points = 11
            for noise in range(10, noise_points):
                # Sweep 10 different noise

                if axis == 'roll':
                    step = math.radians(8)  # 5deg
                    final_noise = float((noise - noise_points / 2) * step)
                    print(final_noise)
                    self.add_angular_noise(final_noise, 0, 0)
                elif axis == 'pitch':
                    step = math.radians(8)  # 5deg
                    final_noise = (noise - noise_points / 2) * step
                    self.add_angular_noise(0, final_noise, 0)
                else:
                    step = math.radians(12)  # 12deg
                    final_noise = (noise - noise_points / 2) * step
                    self.add_angular_noise(0, 0, final_noise)

                for pick in range(10):
                    # At each noise do 10 trials to get some statistics

                    print("Doing pick %i / 10, at %.3f, in %s axis" % (pick, math.degrees(final_noise), axis))
                    dist = 0.1
                    self.open_hand_service()
                    raw_input()
                    self.close_hand_service()

                    # raw_input()
                    self.retrieve(dist)  # Distance in cm
                    # Type the result
                    print("Enter 's' or 'f' for pick success: ")
                    answer = ''
                    while ((answer is not 's') and (answer is not 'f')):
                        answer = str(raw_input())
                    result = [axis, math.degrees(final_noise), answer]
                    results.append(result)
                    self.open_hand_service()

                    raw_input()
                    self.retrieve(-dist)  # Distance in cm

                    # Save the results every time to avoid losing them if anything happens
                    with open('ang_noise_max_results', 'w') as f:
                        write = csv.writer(f)
                        write.writerows(results)

                    # Ask if it is worth doing more trials with the current noise
                    print("Continue checking this noise_point 'y' ? or go to the next 'n'?")
                    keep = raw_input()
                    if str(keep) == "n":
                        break

    def hand_angles(self, hand_gravity, hand_stem, hand_yaw):
        """Brings the hand into the pose (orientation and position)"""

        text = "Going to an IDEAL Starting Position"
        # Place a marker for the text
        self.place_marker_text(self.apple_pos_x, self.apple_pos_y, self.apple_pos_z + 0.5, 0.1,
                               text)

        # Listen to the TF topic
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        # --- Step 1: Assign Gripper's Goal Pose Position
        # Read current pose
        current_pose = self.move_group.get_current_pose().pose
        # Translate the apple's it into the tool's c-frame
        pose_at_world = tf2_geometry_msgs.PoseStamped()
        pose_at_world.pose = current_pose

        # Match the Hand-Gravity angle
        initial_roll = hand_gravity - 180
        roll = math.radians(initial_roll)
        pitch = math.radians(0)
        yaw = math.radians(0)
        q = quaternion_from_euler(roll, pitch, yaw)
	print("New Orientation will be: {}".format(q))

        pose_at_world.pose.orientation.x = q[0]
        pose_at_world.pose.orientation.y = q[1]
        pose_at_world.pose.orientation.z = q[2]
        pose_at_world.pose.orientation.w = q[3]

        # Rotate
        self.move_group.set_pose_target(pose_at_world.pose)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        # Match the Hand-Stem angle
        yaw = math.radians(hand_yaw - 90)
        print("Applying a yaw of %i " % hand_yaw)
        q = quaternion_from_euler(roll, pitch, yaw)

        pose_at_world.pose.orientation.x = q[0]
        pose_at_world.pose.orientation.y = q[1]
        pose_at_world.pose.orientation.z = q[2]
        pose_at_world.pose.orientation.w = q[3]

        # Rotate
        self.move_group.set_pose_target(pose_at_world.pose)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        # 2nd Translate
        pose_at_world.pose.position.x = self.apple_pos_x
        pose_at_world.pose.position.y = self.apple_pos_y
        pose_at_world.pose.position.z = self.apple_pos_z

        pose_at_world.header.frame_id = self.ref_frame
        pose_at_world.header.stamp = rospy.Time(0)

        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            pose_at_tool = tf_buffer.transform(pose_at_world, "palm", rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        pose_at_tool.pose.position.z = pose_at_tool.pose.position.z - 0.04 + 0.035

        # --- Step 3: Convert pose back into reference frame: World's c-frame
        pose_at_tool.header.stamp = rospy.Time(0)

        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            new_pose_at_world = tf_buffer.transform(pose_at_tool, self.ref_frame, rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        # --- Step 4: Finally, move to the new pose
        self.move_group.set_pose_target(new_pose_at_world.pose)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        # Compare poses
        pose_goal = new_pose_at_world.pose
        current_pose = self.move_group.get_current_pose().pose

        # Save pose as an ideal one
        self.ideal_pose = self.move_group.get_current_pose().pose

        # print(current_pose)
        success = all_close(pose_goal, current_pose, 0.01)

        return success

    def adopt_pose(self):
        """Brings the hand into the ideal pose (orientation and position)"""

        text = "Going to an IDEAL Starting Position"
        # Place a marker for the text
        self.place_marker_text(self.apple_pos_x, self.apple_pos_y, self.apple_pos_z + 0.5, 0.1,
                               text)

        # Listen to the TF topic
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        # --- Step 1: Assign Gripper's Goal Pose Position
        # Translate the apple's it into the tool's c-frame
        pose_at_world = tf2_geometry_msgs.PoseStamped()
        pose_at_world.pose = self.ideal_pose

        # Rotate
        self.move_group.set_pose_target(pose_at_world.pose)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        # Compare poses
        pose_goal = pose_at_world.pose
        current_pose = self.move_group.get_current_pose().pose

        # print(current_pose)
        success = all_close(pose_goal, current_pose, 0.01)

        return success

    def baselink_cframe(self):

        # Listen to the tf topic
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        # --- Apple
        # Apple @ World Frame
        apple_at_world = tf2_geometry_msgs.PoseStamped()
        apple_at_world.pose.position.x = self.apple_pos_x
        apple_at_world.pose.position.y = self.apple_pos_y
        apple_at_world.pose.position.z = self.apple_pos_z
        apple_at_world.header.frame_id = self.ref_frame
        apple_at_world.header.stamp = rospy.Time(0)
        # Apple @ Base Link
        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            apple_at_baselink = tf_buffer.transform(apple_at_world, "base_link", rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise
        self.apple_at_baselink = [apple_at_baselink.pose.position.x,
                                  apple_at_baselink.pose.position.y,
                                  apple_at_baselink.pose.position.z]

        # --- Calix
        # Calix @ World Frame
        calix_at_world = tf2_geometry_msgs.PoseStamped()
        calix_at_world.pose.position.x = self.calix[0]
        calix_at_world.pose.position.y = self.calix[1]
        calix_at_world.pose.position.z = self.calix[2]
        calix_at_world.header.frame_id = self.ref_frame
        calix_at_world.header.stamp = rospy.Time(0)
        # Calix @ Base Link
        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            calix_at_baselink = tf_buffer.transform(calix_at_world, "base_link", rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise
        self.calix_at_baselink = [calix_at_baselink.pose.position.x,
                                  calix_at_baselink.pose.position.y,
                                  calix_at_baselink.pose.position.z]

        # --- Stem
        # Stem @ World Frame
        stem_at_world = tf2_geometry_msgs.PoseStamped()
        stem_at_world.pose.position.x = self.stem[0]
        stem_at_world.pose.position.y = self.stem[1]
        stem_at_world.pose.position.z = self.stem[2]
        stem_at_world.header.frame_id = self.ref_frame
        stem_at_world.header.stamp = rospy.Time(0)
        # Calix @ Base Link
        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            stem_at_baselink = tf_buffer.transform(stem_at_world, "base_link", rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise
        self.stem_at_baselink = [stem_at_baselink.pose.position.x,
                                 stem_at_baselink.pose.position.y,
                                 stem_at_baselink.pose.position.z]


        # --- Stem coordinates at the socket
        # @ World frame
        stem_socket_at_world = tf2_geometry_msgs.PoseStamped()
        stem_socket_at_world.pose.position.x = self.stem_socket[0]
        stem_socket_at_world.pose.position.y = self.stem_socket[1]
        stem_socket_at_world.pose.position.z = self.stem_socket[2]
        stem_socket_at_world.header.frame_id = self.ref_frame
        stem_socket_at_world.header.stamp = rospy.Time(0)
        # @ Baselink
        try:
            stem_socket_at_baselink = tf_buffer.transform(stem_socket_at_world, "base_link", rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise
        self.stem_socket_at_baselink = [stem_socket_at_baselink.pose.position.x,
                                        stem_socket_at_baselink.pose.position.y,
                                        stem_socket_at_baselink.pose.position.z]


    ## ... Data saving related functions

    def noise_to_string(self):
        num_formatter = "{0:.2f}"
        output = "_noiseX_" + str(num_formatter.format(self.percent_noise_X)) + \
                 "_noiseY_" + str(num_formatter.format(self.percent_noise_Y)) + \
                 "_noiseZ_" + str(num_formatter.format(self.percent_noise_Z)) + \
                 "_InitPitch_" + str(num_formatter.format(self.initial_pitch_angle)) + \
                 "_InitYaw_" + str(num_formatter.format(self.initial_yaw_angle)) + \
                 "_InitRoll_" + str(num_formatter.format(self.initial_roll_angle)) + \
                 "_PitchNoise_" + str(num_formatter.format(self.pitch_noise_deg)) + \
                 "_RollNoise_" + str(num_formatter.format(self.roll_noise_deg)) + \
                 "_YawNoise_" + str(num_formatter.format(self.yaw_noise_deg))
        return output

    def success_or_failure(self, sof):
        self.success_or_failure_publisher.publish(sof)

    def publish_event(self, event):
        self.event_publisher.publish(event)

    def write_csv(self, data, sub_name):
        header = ["Environment",
                  "f0_proximal", "f0_distal",
                  "f1_proximal", "f1_distal",
                  "f2_proximal", "f2_distal",
                  "slip", "drop", "stem_moved", "success_or_failure",
                  "apple wrt baselink", "calix wrt baselink", "stem wrt baselink", "stem socket wrt baselink", # 11, 12, 13 14
                  "eef_wrt_baselink",  # 15
                  "noise_wrt_tool"  # 16        [x,y,z, roll, pitch, yaw]
                  ]
        # location = '/media/avl/StudyData/Apple Pick Data/Apple Proxy Picks/3 - Winter 22 picks/apple_proxy_pick'
        location = "/home/avl/ur_ws/src/apple_proxy/bag_files/apple_proxy_pick"
        csv_name = location + str(sub_name) + "_metadata.csv"

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

    def get_apple_pose(self):
        apple_pos = [self.apple_pos_x, self.apple_pos_y, self.apple_pos_z]
        return apple_pos

    def get_cartesian_noise_added(self):
        noise = [self.noise_x, self.noise_y, self.noise_z]
        return noise

    def get_angular_noise_added(self):
        noise = [self.yaw_noise_deg, self.pitch_noise_deg, self.roll_noise_deg]
        return noise

    def get_ideal_starting_position(self):
        hand_data = [self.ideal_starting_x, self.ideal_starting_y, self.ideal_starting_z]
        return hand_data

    def get_ideal_starting_orientation(self):
        hand_data = [self.initial_yaw_angle, self.initial_pitch_angle, self.initial_roll_angle]
        return hand_data


def main():
    try:

        # ------------------------------------- Step 1 - Initial Setup -------------------------------------------------
        print("Apple Proxy experiments")
        apple_proxy_experiment = AppleProxyExperiment()

        # Specify whether you will be using the hand or not, to avoid the program from crashing
        apple_proxy_experiment.use_hand = True

        apple_proxy_experiment.open_hand_service()

        # --- Initialize UR5 at home position if needed
        print(" Press 'Enter' to move arm into the original UR5 home position")
        raw_input()
        # apple_proxy_experiment.go_home()

        # --- Bring UR5 into a preliminary position to avoid weird poses
        print(" Press 'Enter' to move arm into a preliminary starting position")
        raw_input()
        apple_proxy_experiment.go_preliminary_position()

        # ---  Bring the robot to the scanned position and normal to the stem
        # print("--- Align Arm with Stem")
        # print("--- Press 'Enter' when ready")
        # apple_proxy_experiment.align_with_stem()

        # --- Read the csv file with all the angles from the real-apple picks [Hand-Stem, Stem-Gravity, Hand-gravity]
        location = '/home/avl/PycharmProjects/AppleProxy/'
        file = 'real_picks_angles_yaw.csv'
        with open(location + file, 'r') as f:
            reader = csv.reader(f)
            angles = list(reader)

        # Add index into each row of angles
        # i = 1
        # for pick in angles:
        #     pick.append(i)
        #     i += 1

        # --- Sort the list according to the Stem-Gravity angle in order to simplify the proxy arrangement
        real_pick_angles = np.array(angles)
        real_pick_angles = real_pick_angles[np.argsort(real_pick_angles[:, 1])]
        # Save list to a csv file
        with open('real_pick_sorted_list', 'w') as f:
            write = csv.writer(f)
            write.writerows(real_pick_angles)

        # ----------------------------- Step 2 - Sweep all the real-apple picks ----------------------------------------
        for j in range(37, len(real_pick_angles)):

            # Avoid the picks in which the robot didn't find a solution within the Proxy
            if j in [0, 1, 16, 17, 27, 31, 37, 41, 43, 48, 53, 54, 56, 57, 58, 64, 74, 75]:
                continue

            pick = real_pick_angles[j]

            # --- Read the angles individually, convert to float, remove decimals
            hand_stem_angle = round(float(pick[0]), 0)
            stem_gravity_angle = round(float(pick[1]), 0)
            hand_gravity_angle = round(float(pick[2]), 0)
            index = round(float(pick[3]), 0)
            yaw = round(float(pick[4]), 0)

            print("\n\n::::::::::::::::::::::: Replicating Real Apple Pick %i :::::::::::::::::::::::::::::" % index)
            print("Hand-Stem angle: %.0f \nStem-Gravity angle: %.0f \nHand-Gravity angle: %.0f \nYaw: %0.f" % (
            hand_stem_angle,
            stem_gravity_angle,
            hand_gravity_angle, yaw))

            # ---  Scan Apple and Stem, and check that Stem-Gravity angle is close enough
            # Stem-Gravity angle reference from the real apple pick
            print("\nThe reference Stem-Gravity angle from the real pick is %.0f deg" % stem_gravity_angle)
            # Stem-Gravity angle from the proxy
            print("The current Stem-Gravity angle in the proxy is %.0f deg" % apple_proxy_experiment.stem_to_gravity)

            difference = apple_proxy_experiment.stem_to_gravity - stem_gravity_angle

            while abs(difference) > 10:         # Only re-scan if the difference is bigger than 10deg
                print("You need to adjust the angle of the stem in the proxy...")
                print("Scan the stem in the proxy")
                apple_proxy_experiment.scan_apple_and_stem()
                stem_to_gravity_at_proxy = apple_proxy_experiment.stem_to_gravity
                print("\nResult: current Stem-Gravity angle in proxy is %.0f deg: " % stem_to_gravity_at_proxy)

                difference = stem_to_gravity_at_proxy - stem_gravity_angle
                print("The difference with the real-apple pick is %.0f deg: " % difference)

            # Place Apple and Stem in RVIZ
            apple_proxy_experiment.place_apple_and_stem()

            # Remind user to push Play in the pendant screen
            print("Push play in the pendant screen and then 'Enter'")
            raw_input()

            # --- Replace Probe with the Gripper
            apple_proxy_experiment.go_preliminary_position()
            print("--- Replace Scanning probe with gripper")
            print("--- Press 'Enter' when ready")
            raw_input()

            # --- Make sure to open hand before approaching apple
            apple_proxy_experiment.open_hand_service()

            # --- Make arm adopt the Hand-Stem and Hand-Gravity angles as in the real-apple picks
            print("\n... Placing arm w.r.t stem and gravity as in the real apple pick")
            angles = apple_proxy_experiment.hand_angles(hand_gravity_angle, hand_stem_angle, yaw)

            # Check Max limits of noise
            # print("--- About to start adding cartesian noise")
            # print("--- Press 'Enter' when ready")
            # raw_input()
            # apple_proxy_experiment.cart_noise()
            # print("--- About to start adding angular noise")
            # print("--- Press 'Enter' when ready")
            # raw_input()
            # apple_proxy_experiment.ang_noise()

            # --- 3rd - Perform 5 picks at each point, by adding random (uniformly distributed) noise at each:
            if angles:
                for noise in range(13):
                    print("\n\n:::: Performing attempt %i/9 of real apple pick %i ::::" % (noise, index))
                    # Perform 5 proxy picks with different noise at each of the reference real-apple picks

                    # --- Add cartesian noise
                    print("\nHit 'Enter' to add cartesian noise")
                    raw_input()
                    # Noise Ranges obtained in the proxy [m]
                    # Original noise ranges - Obtained individually
                    # x_noise_range = [-0.05, -0.04, -0.03, 0.01, 0.02, 0.03]
                    # y_noise_range = [-0.03, -0.02, -0.01, 0.00, 0.01, 0.02, 0.03]
                    # z_noise_range = [-0.025, -0.02, -0.015, -0.01]

                    # The original noise ranges were obtained with isolated noise. However, in reality noise happens
                    # simoultanenously. Thus, it's hard to obtain a combination that would lead to a certain percentage
                    # x_noise_range = [-0.040, -0.025, -0.010, 0.005, 0.020]
                    # y_noise_range = [-0.020, -0.010, +0.000, 0.010, 0.020]
                    # z_noise_range = [-0.020, -0.015, -0.010]

                    x_noise_range = [-0.020, -0.013, -0.005, +0.003, +0.010]
                    y_noise_range = [-0.015, -0.007, +0.000, +0.007, 0.015]
                    z_noise_range = [-0.010, -0.005, 0]

                    rdm_x = np.random.randint(0, 4)
                    rdm_y = np.random.randint(0, 4)
                    rdm_z = np.random.randint(0, 2)

                    x_noise = x_noise_range[rdm_x]  # [m]
                    y_noise = y_noise_range[rdm_y]  # [m]
                    z_noise = z_noise_range[rdm_z]  # [m]

                    print("The cartesian noises added are %.3f, %.3f and %.3f" % (x_noise, y_noise, z_noise))

                    apple_proxy_experiment.add_cartesian_noise(x_noise, y_noise, z_noise)

                    # A final sweep of all the picks without ant noise
                    # apple_proxy_experiment.add_cartesian_noise(0, 0, 0)

                    # --- Add angular noise
                    print("\nHit 'Enter' to add angular noise")
                    raw_input()
                    # Noise Ranges obtained in the proxy [deg]

                    # Original noise ranges - Obtained individually
                    # roll_noise_range = [-40, -32, -24, 24, 32, 40]
                    # pitch_noise_range = [24, 32, 40]

                    # roll_noise_range = [-30, -15, 0, 15, 30]
                    # pitch_noise_range = [-30, -15, 0, 15, 30]
                    # yaw_noise_range = [-30, -15, 0, 15, 30]

                    roll_noise_range = [-10, -5, 0, 5, 10]
                    pitch_noise_range = [-10, -5, 0, 5, 10]
                    yaw_noise_range = [-10, -5, 0, 5, 10]

                    rdm_roll = np.random.randint(0, 4)
                    rdm_pitch = np.random.randint(0, 4)
                    rdm_yaw = np.random.randint(0, 4)

                    roll_noise = math.radians(roll_noise_range[rdm_roll])  # [rad]
                    pitch_noise = math.radians(pitch_noise_range[rdm_pitch])  # [rad]
                    yaw_noise = math.radians(yaw_noise_range[rdm_yaw])  # [rad]

                    print("The angular noises RPY added are %.3f and %.3f and %.3f" % (roll_noise, pitch_noise, yaw_noise))


                    apple_proxy_experiment.add_angular_noise(roll_noise, pitch_noise, yaw_noise)

                    # A final sweep of all the picks without ant noise
                    # apple_proxy_experiment.add_angular_noise(0, 0, 0)

                    # --- Arrange Rosbag file and subscribe to the topics that you want to record
                    rosbag_name = ''
                    sub_name = str(int(index)) + "-" + str(noise)
                    # location = '/media/avl/StudyData/Apple Pick Data/Apple Proxy Picks/3 - Winter 22 picks/apple_proxy_pick'     # Cindy's SSD
                    location = "/home/avl/ur_ws/src/apple_proxy/bag_files/apple_proxy_pick"
                    command = "rosbag record -O " + location \
                              + sub_name + rosbag_name \
                              + " /success_or_failure" \
                                " /applehand/finger1/imu" \
                                " /applehand/finger1/jointstate" \
                                " /applehand/finger1/pressures" \
                                " /applehand/finger2/imu" \
                                " /applehand/finger2/jointstate" \
                                " /applehand/finger2/pressures" \
                                " /applehand/finger3/imu" \
                                " /applehand/finger3/jointstate" \
                                " /applehand/finger3/pressures" \
                                " wrench" \
                                " joint_states" \
                                " /camera/image_raw" \
                                " /apple_trial_events"
                    command = shlex.split(command)
                    rosbag_proc = subprocess.Popen(command)

                    # Create csv with the metadata
                    csv_data = [0] * 20
                    csv_data[0] = "apple_proxy"

                    # --- Publish "0" as event before closing hand
                    time.sleep(1)
                    apple_proxy_experiment.publish_event(0)
                    time.sleep(0.001)

                    # --- Open gripper
                    # print("::: Press 'Enter' to open the gripper :::")
                    # raw_input()
                    apple_proxy_experiment.publish_event(0)
                    time.sleep(0.001)
                    apple_proxy_experiment.publish_event(1)  # Event "1" means Gripper Open
                    time.sleep(0.001)
                    apple_proxy_experiment.open_hand_service()

                    # --- Close gripper
                    print("::: Press 'Enter' to close the gripper :::")
                    raw_input()
                    apple_proxy_experiment.publish_event(1)
                    time.sleep(0.001)
                    apple_proxy_experiment.publish_event(2)  # Event "2" means Gripper Closed
                    time.sleep(0.001)
                    apple_proxy_experiment.close_hand_service()

                    # --- Label GRASP
                    print("F0 proximal link (1-contact, 0-no contact) : ")
                    f0_proximal = ''
                    while ((f0_proximal is not '1') and (f0_proximal is not '0')):
                        f0_proximal = raw_input()
                    csv_data[1] = f0_proximal

                    print("F0 distal link (1-contact, 0-no contact) : ")
                    f0_distal = ''
                    while ((f0_distal is not '1') and (f0_distal is not '0')):
                        f0_distal = raw_input()
                    csv_data[2] = f0_distal

                    print("F1 proximal link (1-contact, 0-no contact) : ")
                    f1_proximal = ''
                    while ((f1_proximal is not '1') and (f1_proximal is not '0')):
                        f1_proximal = raw_input()
                    csv_data[3] = f1_proximal

                    print("F1 distal link (1-contact, 0-no contact) : ")
                    f1_distal = ''
                    while ((f1_distal is not '1') and (f1_distal is not '0')):
                        f1_distal = raw_input()
                    csv_data[4] = f1_distal

                    print("F2 proximal link (1-contact, 0-no contact) : ")
                    f2_proximal = ''
                    while ((f2_proximal is not '1') and (f2_proximal is not '0')):
                        f2_proximal = raw_input()
                    csv_data[5] = f2_proximal

                    print("F2 distal link (1-contact, 0-no contact) : ")
                    f2_distal = ''
                    while ((f2_distal is not '1') and (f2_distal is not '0')):
                        f2_distal = raw_input()
                    csv_data[6] = f2_distal

                    # --- Perform pick by retrieving normally
                    print("\n::: Press 'Enter' to pull the apple :::")
                    raw_input()
                    apple_proxy_experiment.publish_event(2)
                    time.sleep(0.001)
                    apple_proxy_experiment.publish_event(3)  # Event "3" means Pulling Apple
                    time.sleep(0.001)
                    apple_proxy_experiment.retrieve(0.1)
                    time.sleep(0.001)
                    apple_proxy_experiment.publish_event(3)
                    time.sleep(0.001)
                    apple_proxy_experiment.publish_event(4)  # Event "4" means Stop after Pull
                    time.sleep(0.001)

                    # --- Label PICK
                    print("Slip during retrieval(y/n): ")
                    slip = ''
                    while ((slip is not 'y') and (slip is not 'n')):
                        slip = str(raw_input())
                    csv_data[7] = slip

                    print("Drop during retrieval(y/n): ")
                    drop = ''
                    while ((drop is not 'y') and (drop is not 'n')):
                        drop = str(raw_input())
                    csv_data[8] = drop

                    print("Stem shift during retrieval(y/n): ")
                    stem_shift = ''
                    while ((stem_shift is not 'y') and (stem_shift is not 'n')):
                        stem_shift = str(raw_input())
                    csv_data[9] = stem_shift

                    print("==Enter 's' or 'f' for pick success: ")
                    answer = ''
                    while ((answer is not 's') and (answer is not 'f')):
                        answer = str(raw_input())
                    apple_proxy_experiment.success_or_failure(answer)
                    csv_data[10] = answer

                    # --- Save the rest of the metadata and save it
                    # Apple and Stem Ground Truth
                    apple_proxy_experiment.baselink_cframe()
                    csv_data[11] = apple_proxy_experiment.apple_at_baselink
                    csv_data[12] = apple_proxy_experiment.calix_at_baselink
                    csv_data[13] = apple_proxy_experiment.stem_at_baselink
                    csv_data[14] = apple_proxy_experiment.stem_socket_at_baselink
                    # End Effector Pose
                    csv_data[15] = apple_proxy_experiment.pose_at_baselink
                    # Nose that was added
                    noise_at_tool = [x_noise, y_noise, z_noise, roll_noise, pitch_noise, 0]
                    csv_data[16] = noise_at_tool
                    # Final Pose
                    apple_proxy_experiment.write_csv(csv_data, sub_name)

                    # --- Open Gripper
                    print("::: Press 'Enter' to open the gripper :::")
                    raw_input()
                    apple_proxy_experiment.publish_event(4)
                    time.sleep(0.001)
                    apple_proxy_experiment.publish_event(1)  # Event "1" means Gripper Open
                    time.sleep(0.001)
                    apple_proxy_experiment.open_hand_service()

                    time.sleep(1)
                    apple_proxy_experiment.publish_event(1)
                    time.sleep(0.001)

                    # --- Stop rosbag recording after each trial
                    for proc in psutil.process_iter():
                        if "record" in proc.name() and set(command[2:]).issubset(proc.cmdline()):
                            proc.send_signal(subprocess.signal.SIGINT)
                    rosbag_proc.send_signal(subprocess.signal.SIGINT)
                    time.sleep(1)

                    # Return to the ideal pose (simply the original real-apple pick)
                    print("Going back")
                    apple_proxy_experiment.adopt_pose()

            # --- At the end ----
            print("\nHit 'Enter' to try the next apple pick")
            raw_input()

        #
        #   # print("... Step 1: Press `Enter` to move arm into the IDEAL starting Pose")
        #   # raw_input()
        #   # apple_proxy_experiment.go_to_starting_position(i)
        #   # print("\nIdeal Starting Position reached\n")
        #
        #
        #   print("============ Press `Enter` to open the gripper   ====================================")
        #   raw_input()
        #   apple_proxy_experiment.publish_event(0)
        #   time.sleep(0.001)
        #   apple_proxy_experiment.publish_event(1)
        #   apple_proxy_experiment.openHandService()
        #
        #
        #   # print("... Step 2: Press 'Enter' to adjust YAW (fingers w.r.t stem)")
        #   # raw_input()
        #   # apple_proxy_experiment.adjust_yaw()
        #   # print("\nYAW adjusted so the fingers encase the apple\n")
        #   #
        #   #
        #   # print("... Step 3: Press 'Enter' to add CARTESIAN NOISE")
        #   # raw_input()
        #   # apple_proxy_experiment.new_noise(0.0)  # Cartesian noise [m] as parameter
        #   # print("\nCartesian Noise added to the Ideal Starting Position\n")
        #
        #
        #   print("... Step 4: Press 'Enter' to approach apple, and add ANGULAR NOISE")
        #   raw_input()
        #   apple_proxy_experiment.new_go_to_pick_apple(0) # Angular noise [deg] as parameter
        #   print("\nGripper near apple, and with angular noise\n")
        #
        #   print("... Step 5: Press `Enter` to close the gripper")
        #   raw_input()
        #   apple_proxy_experiment.publish_event(1)
        #   time.sleep(0.001)
        #   apple_proxy_experiment.publish_event(2)
        #   time.sleep(0.001)
        #   apple_proxy_experiment.closeHandService()
        #
        #
        #
        #
        #   print("... Step 6: Press 'Enter' to retrieve normally")
        #   raw_input()
        #   apple_proxy_experiment.publish_event(2)
        #   time.sleep(0.001)
        #   apple_proxy_experiment.publish_event(3)
        #   time.sleep(0.001)
        #   apple_proxy_experiment.new_retrieve()
        #
        #   apple_proxy_experiment.publish_event(3)
        #   time.sleep(0.001)
        #   apple_proxy_experiment.publish_event(4)
        #   time.sleep(0.001)
        #
        #
        #
        #
        #   # Final Step: Get the rest of the metadata and save it
        #   apple_pose = apple_proxy_experiment.get_apple_pose()
        #   csv_data[11] = apple_pose[0]
        #   csv_data[12] = apple_pose[1]
        #   csv_data[13] = apple_pose[2]
        #
        #   starting_pose = apple_proxy_experiment.get_ideal_starting_position()
        #   csv_data[14] = starting_pose[0]
        #   csv_data[15] = starting_pose[1]
        #   csv_data[16] = starting_pose[2]
        #
        #   cartesian_noise = apple_proxy_experiment.get_cartesian_noise_added()
        #   csv_data[17] = cartesian_noise[0]
        #   csv_data[18] = cartesian_noise[1]
        #   csv_data[19] = cartesian_noise[2]
        #
        #   starting_orientation = apple_proxy_experiment.get_ideal_starting_orientation()
        #   csv_data[20] = starting_orientation[0]
        #   csv_data[21] = starting_orientation[1]
        #   csv_data[22] = starting_orientation[2]
        #
        #   angular_noise = apple_proxy_experiment.get_angular_noise_added()
        #   csv_data[23] = angular_noise[0]
        #   csv_data[24] = angular_noise[1]
        #   csv_data[25] = angular_noise[2]
        #
        #   apple_proxy_experiment.write_csv(csv_data)
        #
        #
        #   print("\n... Press 'Enter' to start next trial\n\n")
        #   raw_input()

        print("================================= Apple experiment complete! ===============================")


    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
