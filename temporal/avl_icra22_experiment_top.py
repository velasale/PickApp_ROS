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
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler
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
    
    #group_name = "panda_arm"
    group_name = "manipulator"
    #group_name = "endeffector"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
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
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

    # Parameters of the Experiment
    self.gripper_height = 0.175  # Distance from the fingers to the TCP
    self.delta_z = 0.058


  def go_to_home(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group
    current_joints = move_group.get_current_joint_values()
    print ("Initial Joints State: ", current_joints)

    # The following are the initial
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = -pi/2
    joint_goal[1] = 0
    joint_goal[2] = - pi * 145 / 180
    joint_goal[3] = - pi * 3 / 4
    joint_goal[4] = - pi/2
    joint_goal[5] = 0

    move_group.go(joint_goal, wait=True)
    move_group.stop()

    # For testing:
    current_joints = move_group.get_current_joint_values()
    print("Final Joints State: ", current_joints)

    return all_close(joint_goal, current_joints, 0.01)


  def go_to_prelim_start(self):
    move_group = self.move_group
    current_joints = move_group.get_current_joint_values()
    print("Initial Joints State: ", current_joints)

    # The following are the initial joints positions
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = - pi / 6
    joint_goal[1] = - 77 * pi / 180
    joint_goal[2] = + 20 * pi / 180
    joint_goal[3] = - 40 * pi / 180
    joint_goal[4] = - pi / 2
    joint_goal[5] = 0

    move_group.go(joint_goal, wait=True)
    move_group.stop()

    # For testing:
    current_joints = move_group.get_current_joint_values()
    print("Final Joints State: ", current_joints)

    return all_close(joint_goal, current_joints, 0.01)


  def go_to_starting_position(self):
    move_group = self.move_group
    current_pose = self.move_group.get_current_pose().pose
    print ("Current End Tool's Pose: ", current_pose)

    # Step 2: Adjust the z position, by keeping the orientation of the end effector
    #pose_goal = geometry_msgs.msg.Pose()
    pose_goal = self.move_group.get_current_pose().pose

    current_pose_z = pose_goal.position.z
    pose_goal.position.z = current_pose_z - 0.12

    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    move_group.stop()

    current_pose = self.move_group.get_current_pose().pose
    print ("Final End Tool's Pose: ", current_pose)

    return all_close(pose_goal, current_pose, 0.01)


  def add_angular_noise(self, noise_range):
    move_group = self.move_group
    pose_goal = self.move_group.get_current_pose().pose
    print("Current End Tool's Pose: ", pose_goal)

    # --- Step 1: Add noise to the Initial Pose
    # Add noise to the ROLL Angle
    roll_noise_deg = noise_range * random() - noise_range / 2
    #roll_noise_deg = 0.0
    roll_noise_rad = roll_noise_deg * pi / 180
    roll_angle = 0.0 + roll_noise_rad

    # Add noise to the PITCH Angle
    self.pitch_noise_deg = noise_range * random() - noise_range / 2
    #self.pitch_noise_deg = 25
    pitch_noise_rad = self.pitch_noise_deg * pi / 180
    pitch_angle = pi/2 + pitch_noise_rad

    # Add noise to the YAW Angle
    yaw_noise_deg = noise_range * random() - noise_range / 2
    #yaw_noise_deg = 0
    yaw_noise_rad = yaw_noise_deg * pi / 180
    yaw_angle = 0 + yaw_noise_rad

    print('angular noise = ', roll_angle, pitch_angle, yaw_angle)

    print('angular noise = ', roll_noise_deg, self.pitch_noise_deg, yaw_noise_deg)

    # --- Step2: Obtain the quaternion
    quaternion = quaternion_from_euler(roll_angle, pitch_angle, yaw_angle)
    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]

    # --- Step 3: Adjust the EndTool effector coordinates after the noise was added

    #pose_goal.position.x = self.apple_pos_x
    #pose_goal.position.y = self.apple_pos_y
    #pose_goal.position.z = self.apple_pos_z

    self.pos_x = pose_goal.position.x
    self.pos_z = pose_goal.position.z

    move_group.set_pose_target(pose_goal)

    # --- Step3: Perform the movement with moveit!
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()


    current_pose = self.move_group.get_current_pose().pose
    # Save this pose in a global variable, to come back to it after picking the apple
    self.previous_pose = pose_goal

    print("Final End Tool's Pose after adding Noise: ", current_pose)

    return all_close(pose_goal, current_pose, 0.01)


  def go_to_pick_apple(self):
    move_group = self.move_group
    pose_goal = self.move_group.get_current_pose().pose

    ## Pose Position
    #pose_goal.position.x = +0.47 + self.gripper_height * math.sin(self.pitch_noise_deg*pi/180)
    #pose_goal.position.y = -0.012

    current_z_pose = pose_goal.position.z

    pose_goal.position.z = current_z_pose - 0.12

    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    current_pose = self.move_group.get_current_pose().pose

    #Returns a check if the current pose is within a certain tolerance from the pose_goal
    return all_close(pose_goal, current_pose, 0.01)


  def retrieve(self):
    move_group = self.move_group

    pose_goal = self.previous_pose

    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    current_pose = self.move_group.get_current_pose().pose

    # Returns a check if the current pose is within a certain tolerance from the pose_goal
    return all_close(pose_goal, current_pose, 0.01)




def main():
  try:
    print ""
    print "----------------------------------------------------------"
    print " Apple Proxy Experiment with UR5 and Robotiq 2f-85 gripper"
    print "----------------------------------------------------------"
    print "Press Ctrl-D to exit at any time"
    print ""
    print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ======"
    raw_input()
    appleProxyExperiment = AppleProxyExperiment()


    print "============ Press `Enter` to move arm into the original UR5 home position ==============="
    raw_input()
    #appleProxyExperiment.go_to_home()




    apples_to_pick = 10
    for i in range(apples_to_pick):

      print "============ Press `Enter` to move arm into the IDEAL starting position and pose ========="
      raw_input()
      appleProxyExperiment.go_to_prelim_start()
      print("Ideal Starting Position reached")

      print "=========== Press 'Enter' to add NOISE to the ideal starting position and pose ==========="
      raw_input()
      appleProxyExperiment.add_angular_noise(10)
      print("Noise added to the Ideal Starting Position ")

      print "============ Press `Enter` to go and pick the apple   ===================================="
      raw_input()
      appleProxyExperiment.go_to_pick_apple()


      print "============ Press 'Enter' to retrieve normally =========================================="
      raw_input()
      appleProxyExperiment.retrieve()

      # Time in seconds
      time.sleep(1)
      print ("Apple No.", i)



    print "============ Python tutorial demo complete!"


  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

