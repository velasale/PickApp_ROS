#!/usr/bin/env python
"""
Code for picking an apple from the proxy, by placing the end effector (hand) at a position on the surface of a sphere
around the apple.

Alejandro Velasquez
velasale@oregonstate.edu

"""
import math
import sys
import copy
import rospy
import numpy as np
import time
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from random import random
import os
import tf

import tf2_ros
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped

from tf.transformations import euler_from_quaternion, quaternion_about_axis
import subprocess, shlex, psutil
from visualization_msgs.msg import Marker, MarkerArray
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler
from numpy import pi, cos, sin, arccos, arange
import matplotlib.pyplot as pp
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Polygon

from std_msgs.msg import String
from std_msgs.msg import Int32
import csv

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
    #group_name = "endeffector"
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
    print "============ Planning frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print "============ End effector link: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""


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

    # Parameters of the Experiment
    #self.gripper_height = 0.132   # Distance from the middle of the fingers to the TCP (Tool Center Point)
    self.delta_z = 0.15          # Distance to move the gripper towards the apple

    self.apple_pos_x = 0.41      # Apple position in x-axis [m]
    self.apple_pos_y = - 0.115         # Apple position in y-axis [m]
    self.apple_pos_z = 0.455       # Apple position in z-axis [m] : Height from the + Distance from table to,
    self.sphereRadius = 0.25

    # this are updated in the go_to_starting_position function
    self.initial_roll_angle = 0
    self.initial_pitch_angle = 0
    self.initial_yaw_angle = 0

    # these are no longer used since the pitch yaw and roll starting positions come from the spherical sampling rather than being user defined
    # self.initial_roll = (pi / 180) * self.initial_roll_angle  # Roll of the initial ideal position [deg]
    # self.initial_pitch = (pi / 180) * self.initial_pitch_angle  # Pitch of the initial ideal position [rad]
    # self.initial_yaw = ( pi / 180) * self.initial_yaw_angle  # Yaw of the initial ideal position [deg]         # Yaw of the initial ideal position [deg]

    # To be measured from drooping fingers in 90 degree position over apple
    self.max_noise_x = 2.5/100
    self.max_noise_y = 2.5/100
    self.max_noise_z = 1.15/100

    #percent noise should be entered in decimal eg 90% = .90
    # negative percentage allowed
    #used for specific amount of noise in each axis rather than a random combination
    self.percent_noise_X = 0.5 # do 1 with 50 and 1 with 100
    self.percent_noise_Y = 0.5
    self.percent_noise_Z = - 0.5  # -.5 seems like its the lowest it can go bc it starts to crash into the apple

    self.noise_x = self.max_noise_x * self.percent_noise_X
    self.noise_y = self.max_noise_y * self.percent_noise_Y
    self.noise_z = self.max_noise_z * self.percent_noise_Z

    # Used in the add angular noise function to use in the naming function noist_toString
    self.angular_noise_range = 5    # do 5 and 10
    self.roll_noise_deg = 1 * (self.angular_noise_range * random() - self.angular_noise_range / 2)
    # Add noise to the PITCH Angle
    self.pitch_noise_deg = 1 * (self.angular_noise_range * random() - self.angular_noise_range / 2)
    # Add noise to the YAW Angle
    self.yaw_noise_deg = 1 * (self.angular_noise_range * random() - self.angular_noise_range / 2)

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





    #updated in the add angular noise function to use in the naming function noist_toString
    self.total_roll = 0
    self.total_pitch = 0
    self.total_yaw = 0


    #Vector that points from the initial starting position, towards the center of the apple
    self.vector = [0,0,0]
    self.cross_vector = [0,0,0]

    self.angular_noise = 10 * pi / 180    # Angular noise in [rad]
    self.cartesian_noise = 0.04          # Cartesian noise in [cm]

    # Variables for the markers (sampling sphere, apple, ideal starting points)
    self.marker_id = 1
    self.proxy_markers = MarkerArray()
    self.markerPublisher = rospy.Publisher('balloons', MarkerArray, queue_size = 1000)
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

    self.azimuth = 0
    self.elevation = 0

    # trial number is used later in the program to make sure the csv and the corresponding bag file have the same pick #
    self.trial_number = 0


## ... Hand Related Functions...

  def closeHandService(self):
    os.system("rosservice call /applehand/close_hand")

  def openHandService(self):
    os.system("rosservice call /applehand/open_hand")

  def relaxHandService(self):
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

  def place_marker(self, r, g, b, a, x, y, z, scale):
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


## ... Arm related functions

  def go_to_home(self):
    current_joints = self.move_group.get_current_joint_values()
    print ("Initial Joints State: ", current_joints)

    # Initial / Default joint values
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[0] = - pi/2
    joint_goal[1] = 0
    joint_goal[2] = - pi * 145 / 180
    joint_goal[3] = - pi * 3 / 4
    joint_goal[4] = - pi/2
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

    # Place a marker for the apple
    self.place_marker(1, 0, 0, 1.0, self.apple_pos_x, self.apple_pos_y, self.apple_pos_z, 0.08)
    # Place a marker for the sampling sphere
    self.place_marker(0, 1, 0, 0.2, self.apple_pos_x, self.apple_pos_y, self.apple_pos_z, self.sphereRadius * 2)
    # Place a marker for the text
    self.place_marker_text(self.apple_pos_x, self.apple_pos_y, self.apple_pos_z + 0.5, 0.1, "Going to Preliminary Starting Position")

    # The following are the initial joints positions
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[0] = - 178 * pi / 180
    joint_goal[1] = - 95 * pi / 180
    joint_goal[2] = - 35 * pi / 180
    joint_goal[3] = - 138 * pi / 180
    joint_goal[4] = + 90 * pi / 180
    joint_goal[5] = 0

    self.move_group.go(joint_goal, wait=True)
    self.move_group.stop()

    # Print for debugging:
    current_joints = self.move_group.get_current_joint_values()
    print("Final Joints State: ", current_joints)

    return all_close(joint_goal, current_joints, 0.01)

  def go_to_starting_position(self, index):
    """This function takes the gripper to the IDEAL starting position, before adding noise

    """

    text = "Going to an IDEAL Starting Position # " + str(index)
    # Place a marker for the text
    self.place_marker_text(self.apple_pos_x, self.apple_pos_y, self.apple_pos_z + 0.5, 0.1,
                           text)

    current_pose = self.move_group.get_current_pose().pose

    # A. Sphere center location = apple's location
    # Center of Sphere = Location of apple
    h = self.apple_pos_x
    k = self.apple_pos_y
    l = self.apple_pos_z

    x = self.x_coord[index] + h
    y = self.y_coord[index] + k
    z = self.z_coord[index] + l

    self.azimuth = math.atan2(self.y_coord[index], self.x_coord[index]) * 180 / pi

    # Step 1 - Get the vector pointing to the center of the sphere
    delta_x = x - h
    delta_y = y - k
    delta_z = z - l
    length = math.sqrt(delta_x ** 2 + delta_y ** 2 + delta_z ** 2)
    # print("deltas", delta_x, delta_y, delta_z, length)

    # Step 2 - Make the cross product between that vector and the vector normal to the palm "z" to obtain the rotation vector
    V1 = [0, 0, 1]
    V2 = [-delta_x / length, -delta_y / length, -delta_z / length]  # Normalize it
    V3 = np.cross(V1, V2)

    self.vector = [delta_x, delta_y, delta_z]
    self.cross_vector = V3

    # Step 3 - Make the dot product to obtain the angle of rotation
    dot = np.dot(V1, V2)
    angle = math.acos(dot)

    # Step 4 - Obtain the quaternion from the single axis rotation
    q = quaternion_about_axis(angle, V3)

    pose_goal = self.move_group.get_current_pose().pose

    pose_goal.orientation.x = q[0]
    pose_goal.orientation.y = q[1]
    pose_goal.orientation.z = q[2]
    pose_goal.orientation.w = q[3]

    roll, pitch, yaw = euler_from_quaternion(q)

    # Cartesian adjustment of the gripper's position, after PITCH rotation so the palm's normal vector points towards the apple
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z

    self.initial_pitch_angle = pitch / pi * 180
    self.initial_yaw_angle = yaw / pi * 180
    self.initial_roll_angle = roll / pi * 180

    self.ideal_starting_x = pose_goal.position.x
    self.ideal_starting_y = pose_goal.position.y
    self.ideal_starting_z = pose_goal.position.z

    self.move_group.set_pose_target(pose_goal)
    plan = self.move_group.go(wait=True)
    self.move_group.stop()

    # Get the current pose to compare it
    current_pose = self.move_group.get_current_pose().pose

    # Save this pose in a global variable, to come back to it after picking the apple
    self.previous_pose = current_pose

    # Place a blue dot in the sphere's surface to keep track of all the sampled points
    self.place_marker(0, 0, 1, 1, x, y, z, 0.02)

    success = all_close(pose_goal, current_pose, 0.01)
    self.pose_starts.append(success)
    print("Pose Starts history", self.pose_starts)

    return success

  def adjust_yaw(self):
    """
    This function adjusts the YAW of the end toll, so the hand fingers avoid jamming into the STEM.
    Step 1: Transform the stem's vector from the world into the tool's local frame
    Step 2: Cross product the stem's vector in the new frame, with the z axis, to obtain the vector normal to the plane
            created by thee two vector.
    Step 3: Dot product between the rotating vector abd the x-axis, subtract pi/2, and obtain the angle to rotate YAW
    Step 4: Rotate YAW and transform it into the world frame
    """

    # Place a marker for the text
    self.place_marker_text(self.apple_pos_x, self.apple_pos_y, self.apple_pos_z + 0.5, 0.1,
                           "Adjust YAW... so the fingers encage the apple")

    # --- Step 1: Read the pose from the "Base_link" into "Tool0"
    # **Assuming /tf2 topic is being broadcasted
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    # Point A
    pose_stamped_A = tf2_geometry_msgs.PoseStamped()
    pose_stamped_A.pose.position.x = 0
    pose_stamped_A.pose.position.y = 0
    pose_stamped_A.pose.position.z = 0

    pose_stamped_A.header.frame_id = "base_link"
    pose_stamped_A.header.stamp = rospy.Time(0)

    try:
      # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
      output_pose_stamped_A = tf_buffer.transform(pose_stamped_A, "tool0", rospy.Duration(1))
      #print("Point A in Tool0: ", output_pose_stamped_A.pose)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      raise

    # Point B
    pose_stamped_B = tf2_geometry_msgs.PoseStamped()
    pose_stamped_B.pose.position.x = 0
    pose_stamped_B.pose.position.y = 0
    pose_stamped_B.pose.position.z = 1

    pose_stamped_B.header.frame_id = "base_link"
    pose_stamped_B.header.stamp = rospy.Time(0)

    try:
      # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
      output_pose_stamped_B = tf_buffer.transform(pose_stamped_B, "tool0", rospy.Duration(1))
      #print("Point B in Tool0: ", output_pose_stamped_B.pose)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      raise

    C = [output_pose_stamped_A.pose.position.x, output_pose_stamped_A.pose.position.y, output_pose_stamped_A.pose.position.z]
    D = [output_pose_stamped_B.pose.position.x, output_pose_stamped_B.pose.position.y, output_pose_stamped_B.pose.position.z]

    E = np.subtract(D,C)
    #print ("\nStem vector in tool", E)
    length_E = math.sqrt(E[0]**2 + E[1]**2 + E[2]**2)
    E[0] = E[0]/length_E
    E[1] = E[1]/length_E
    E[2] = E[2]/length_E

    # --- Step 2 ---

    Z = [0, 0, 1]
    F = np.cross(E, Z)
    length_F = math.sqrt(F[0] ** 2 + F[1] ** 2 + F[2] ** 2)
    F[0] = F[0] / length_F
    F[1] = F[1] / length_F
    F[2] = F[2] / length_F

    self.elevation = 180 - math.acos(np.dot(E,Z)) * 180 / pi
    print ('\n The elevation angle is (angle between stem and local z) and azimuth are', self.elevation, self.azimuth)

    dot = np.dot([1,0,0],F)
    angle = math.acos(dot) - pi /2

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
    self.pose_yaws.append(success)
    print("Pose Yaws history", self.pose_yaws)

    return success

  def new_noise(self, cartesian_noise):

    # Place a marker for the text
    self.place_marker_text(self.apple_pos_x, self.apple_pos_y, self.apple_pos_z + 0.5, 0.1,
                           "Add CARTESIAN noise in the Tool's local frame")

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
    second_pose = output_pose_stamped.pose
    #second_pose.position.x = second_pose.position.x + 1 * (random() * cartesian_noise - cartesian_noise / 2)
    #second_pose.position.y = second_pose.position.y + 1 * (random() * cartesian_noise - cartesian_noise / 2)
    #second_pose.position.z = second_pose.position.z + 1 * (random() * cartesian_noise - cartesian_noise / 2)
    second_pose.position.x = second_pose.position.x + 1 * self.noise_x
    second_pose.position.y = second_pose.position.y + 1 * self.noise_y
    second_pose.position.z = second_pose.position.z + 1 * self.noise_z

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
    self.pose_noises.append(success)
    print("Pose Noises history", self.pose_noises)

    return success

  def new_go_to_pick_apple(self, angular_noise_deg):

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
    pose_stamped.header.frame_id = "world"
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
    #second_pose.position.z = second_pose.position.z + self.delta_z
    second_pose.position.z = second_pose.position.z + 0

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
      output_pose_stamped = tf_buffer.transform(sec_pose_stamped, "world", rospy.Duration(1))
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

  def new_retrieve(self):

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
    pose_stamped.header.frame_id = "world"
    pose_stamped.header.stamp = rospy.Time(0)

    try:
      # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
      output_pose_stamped = tf_buffer.transform(pose_stamped, "tool0", rospy.Duration(1))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      raise

    # --- Step 2: Add distance to retrieve
    second_pose = output_pose_stamped.pose
    #second_pose.position.z = second_pose.position.z - self.delta_z - 0.025
    second_pose.position.z = second_pose.position.z - 0.1



    # --- Step 3: Convert the pose back into the "Base_Link" reference frame
    sec_pose_stamped = tf2_geometry_msgs.PoseStamped()
    sec_pose_stamped.pose = second_pose
    sec_pose_stamped.header.frame_id = "tool0"
    sec_pose_stamped.header.stamp = rospy.Time(0)

    try:
      # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
      output_pose_stamped = tf_buffer.transform(sec_pose_stamped, "world", rospy.Duration(1))
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
    self.pose_retrievals.append(success)
    print("Pose Retrievals history", self.pose_retrievals)

    return success

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
      if x[i] < 0:    # To get only half sphere
        if z[i] > 0:    # To get only one quarter of the sphere
            if y[i] > 0:    # To get only one eight of the sphere
              self.x_coord.append(x[i] * self.sphereRadius)
              self.y_coord.append(y[i] * self.sphereRadius)
              self.z_coord.append(z[i] * self.sphereRadius)


## ... Data saving related functions

  def noise_toString(self):
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

  def write_csv(self, data):
        header = ["stem_orientation", "f0_proximal", "f0_distal", "f1_proximal", "f1_distal", "f2_proximal",
                  "f2_distal", "slip", "drop", "stem_moved", "success_or_failure", "apple_x", "apple_y", "apple_z",
                  "hand_ideal_starting_pos_x", "hand_ideal_starting_pos_y", "hand_ideal_starting_pos_z",
                  "noise_x", "noise_y", "noise_z",
                  "hand_init_yaw", "hand_init_pitch", "hand_init_roll",
                  "noise_yaw", "noise_pitch", "noise_roll"]
        csv_name = "/home/avl/ur_ws/src/apple_proxy/bag_files/apple_proxy_pick" + str(
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
    print ""
    print "----------------------------------------------------------"
    print " Apple Proxy Experiment with UR5 and Lisa's hand          "
    print "----------------------------------------------------------"
    print "Press Ctrl-D to exit at any time"
    print ""
    print "============ Press `Enter` to begin the Experiment by setting up the moveit_commander ======"
    raw_input()
    apple_proxy_experiment = AppleProxyExperiment()


    # print "============ Press `Enter` to move arm into the original UR5 home position ==============="
    # raw_input()
    #appleProxyExperiment.go_to_home()


    # print "============ Press 'Enter' to move arm into a preliminary starting position =============="
    # raw_input()
    #apple_proxy_experiment.go_to_prelim_start()


    apple_proxy_experiment.point_sampling()
    apples_to_pick = len(apple_proxy_experiment.x_coord)

    for i in range(65, 100):
      apple_proxy_experiment.update_trial_num(i)

      # sets up rosbag to subscribe to all topics for each trial
      rosbag_name = ''
      #rosbag_name = apple_proxy_experiment.noise_toString()     # Could be missleading

      command = "rosbag record -O " + "/home/avl/ur_ws/src/apple_proxy/bag_files/apple_proxy_pick" + str(
        i) + rosbag_name + " /success_or_failure /applehand/finger1/imu /applehand/finger1/jointstate /applehand/finger1/pressures /applehand/finger2/imu /applehand/finger2/jointstate /applehand/finger2/pressures /applehand/finger3/imu /applehand/finger3/jointstate /applehand/finger3/pressures wrench joint_states /camera/image_raw /apple_trial_events"
      command = shlex.split(command)
      rosbag_proc = subprocess.Popen(command)

      time.sleep(1)
      apple_proxy_experiment.publish_event(0)
      time.sleep(0.001)


      text = "------------------------- Trial No. " + str(i) + " -------------------------"
      print(text)


      # print("... Step 1: Press `Enter` to move arm into the IDEAL starting Pose")
      # raw_input()
      # apple_proxy_experiment.go_to_starting_position(i)
      # print("\nIdeal Starting Position reached\n")


      print("============ Press `Enter` to open the gripper   ====================================")
      raw_input()
      apple_proxy_experiment.publish_event(0)
      time.sleep(0.001)
      apple_proxy_experiment.publish_event(1)
      apple_proxy_experiment.openHandService()


      # print("... Step 2: Press 'Enter' to adjust YAW (fingers w.r.t stem)")
      # raw_input()
      # apple_proxy_experiment.adjust_yaw()
      # print("\nYAW adjusted so the fingers encase the apple\n")
      #
      #
      # print("... Step 3: Press 'Enter' to add CARTESIAN NOISE")
      # raw_input()
      # apple_proxy_experiment.new_noise(0.0)  # Cartesian noise [m] as parameter
      # print("\nCartesian Noise added to the Ideal Starting Position\n")


      print("... Step 4: Press 'Enter' to approach apple, and add ANGULAR NOISE")
      raw_input()
      apple_proxy_experiment.new_go_to_pick_apple(0) # Angular noise [deg] as parameter
      print("\nGripper near apple, and with angular noise\n")

      print("... Step 5: Press `Enter` to close the gripper")
      raw_input()
      apple_proxy_experiment.publish_event(1)
      time.sleep(0.001)
      apple_proxy_experiment.publish_event(2)
      time.sleep(0.001)
      apple_proxy_experiment.closeHandService()

      csv_data = [0] * apples_to_pick
      csv_data[0] = "apple_proxy"

      # Next: Evaluate the Grasp quality
      print("F0 proximal link(1-contact, 0-no contact) : ")
      f0_proximal = ''
      while ((f0_proximal is not '1') and (f0_proximal is not '0')):
        f0_proximal = raw_input()
      csv_data[1] = f0_proximal

      print("F0 distal link(1-contact, 0-no contact) : ")
      f0_distal = ''
      while((f0_distal is not '1') and (f0_distal is not '0')):
        f0_distal = raw_input()
      csv_data[2] = f0_distal

      print("F1 proximal link(1-contact, 0-no contact) : ")
      f1_proximal = ''
      while((f1_proximal is not '1') and (f1_proximal is not '0')):
        f1_proximal = raw_input()
      csv_data[3] = f1_proximal

      print("F1 distal link(1-contact, 0-no contact) : ")
      f1_distal = ''
      while((f1_distal is not '1') and (f1_distal is not '0')):
        f1_distal = raw_input()
      csv_data[4] = f1_distal

      print("F2 proximal link(1-contact, 0-no contact) : ")
      f2_proximal = ''
      while((f2_proximal is not '1') and (f2_proximal is not '0')):
        f2_proximal = raw_input()
      csv_data[5] = f2_proximal

      print("F2 distal link(1-contact, 0-no contact) : ")
      f2_distal = ''
      while((f2_distal is not '1') and (f2_distal is not '0')):
        f2_distal = raw_input()
      csv_data[6] = f2_distal


      print("... Step 6: Press 'Enter' to retrieve normally")
      raw_input()
      apple_proxy_experiment.publish_event(2)
      time.sleep(0.001)
      apple_proxy_experiment.publish_event(3)
      time.sleep(0.001)
      apple_proxy_experiment.new_retrieve()

      apple_proxy_experiment.publish_event(3)
      time.sleep(0.001)
      apple_proxy_experiment.publish_event(4)
      time.sleep(0.001)



      # Evaluate the pick quality / result
      print("Slip during retrieval(y/n): ")
      slip = ''
      while((slip is not 'y') and (slip is not 'n')):
        slip = str(raw_input())
      csv_data[7] = slip

      print("Drop during retrieval(y/n): ")
      drop = ''
      while((drop is not 'y') and (drop is not 'n')):
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


      # Final Step: Get the rest of the metadata and save it
      apple_pose = apple_proxy_experiment.get_apple_pose()
      csv_data[11] = apple_pose[0]
      csv_data[12] = apple_pose[1]
      csv_data[13] = apple_pose[2]

      starting_pose = apple_proxy_experiment.get_ideal_starting_position()
      csv_data[14] = starting_pose[0]
      csv_data[15] = starting_pose[1]
      csv_data[16] = starting_pose[2]

      cartesian_noise = apple_proxy_experiment.get_cartesian_noise_added()
      csv_data[17] = cartesian_noise[0]
      csv_data[18] = cartesian_noise[1]
      csv_data[19] = cartesian_noise[2]

      starting_orientation = apple_proxy_experiment.get_ideal_starting_orientation()
      csv_data[20] = starting_orientation[0]
      csv_data[21] = starting_orientation[1]
      csv_data[22] = starting_orientation[2]

      angular_noise = apple_proxy_experiment.get_angular_noise_added()
      csv_data[23] = angular_noise[0]
      csv_data[24] = angular_noise[1]
      csv_data[25] = angular_noise[2]

      apple_proxy_experiment.write_csv(csv_data)

      # TO DO: This is where we want to include the code to open the gripper
      print("============ Press `Enter` to open the gripper   ====================================")
      raw_input()
      apple_proxy_experiment.publish_event(4)
      time.sleep(0.001)
      apple_proxy_experiment.publish_event(1)
      time.sleep(0.001)
      apple_proxy_experiment.openHandService()

      time.sleep(4)
      apple_proxy_experiment.publish_event(1)
      time.sleep(0.001)

      # Stop rosbag recording after each trial
      for proc in psutil.process_iter():
        if "record" in proc.name() and set(command[2:]).issubset(proc.cmdline()):
          proc.send_signal(subprocess.signal.SIGINT)

      rosbag_proc.send_signal(subprocess.signal.SIGINT)

      time.sleep(1)


      print("\n... Press 'Enter' to start next trial\n\n")
      raw_input()


    print "================================= Apple experiment complete! ==============================="


  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

