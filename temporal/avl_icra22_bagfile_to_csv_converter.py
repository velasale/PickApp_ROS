"""
This file opens all the bagifles that obtain the data from the experiments, and creates a csv for each topic.
"""

import bagpy
import pandas as pd
from bagpy import bagreader
import matplotlib.pyplot as plt
import math
import numpy as np
import seaborn as sns
import csv
import os
from scipy.ndimage.filters import gaussian_filter

sns.set()  # Setting seaborn as default style even if use only matplotlib

root = '/media/avl/StudyData/ApplePicking Data/'

# For approach 1 uncomment these lines
# approach_folder = '1 - Apple Proxy with planar approach/'
# experiment_subfolders = ['all_bag_files']

# # For approach 2 uncomment these lines
# approach_folder = '2 - Apple Proxy with spherical approach/'
# # experiment_subfolders = ['appleProxy_spherical_withNoise (-0.5) (Round1) (Sep04)',
# #                          'appleProxy_spherical_withNoise (-1.0 and 10deg) (Round1) (Sep06)',
# #                          'appleProxy_spherical_withNoise (+0.5) (Round1) (Sep04)',
# #                          'appleProxy_spherical_withNoise (+0.5) (Round2) (Sep04)',
# #                          'appleProxy_spherical_withNoise (+0.5) (Round3) (Sep04)',
# #                          'appleProxy_spherical_withoutNoise (Round1) (Sep03)',
# #                          'appleProxy_spherical_withoutNoise (Round2) (Sep06)',
# #                          'appleProxy_spherical_withoutNoise (Round3) (Sep06)']
# experiment_subfolders = ['appleProxy_spherical_withoutNoise (Round1) (Sep03)',
#                          'appleProxy_spherical_withoutNoise (Round2) (Sep06)',
#                          'appleProxy_spherical_withoutNoise (Round3) (Sep06)']

# For approach 3 uncomment these lines
approach_folder = '3 - Real Apple with straight approach/'
experiment_subfolders = ['day1_august_25_2021',
                         'day2_august_26_2021']



for subfolder in experiment_subfolders:
    print('\nThe subfolder is:\n', subfolder)
    location = root + approach_folder + subfolder

    counter = 0  # Keep a count of all files found
    bagfiles = []  # List to store all files found at location

    # ------------------------------------- Step 1: Search the sub folder for the bagfiles -----------------------------
    for file in os.listdir(location):
        try:
            if file.endswith(".bag"):
                bagfiles.append(str(file))
                counter = counter + 1
        except Exceptio as e:
            raise e
            print("No files found here!\n")
    print("Total files found:\n", counter)
    #print(bagfiles)

    # -------------------------- Step 2: Go through each bagfile and create a csv file for each topic ------------------
    for i in range(counter):
        file_to_open = location + '/' + bagfiles[i]
        print("File to open:\n", file_to_open)
        b = bagreader(file_to_open)
        data_arm_wrench = b.message_by_topic('wrench')
        data_arm_joints = b.message_by_topic('joint_states')
        data_f1_imu = b.message_by_topic('/applehand/finger1/imu')
        data_f1_joint = b.message_by_topic('/applehand/finger1/jointstate')
        data_f2_imu = b.message_by_topic('/applehand/finger2/imu')
        data_f2_joint = b.message_by_topic('/applehand/finger2/jointstate')
        data_f3_imu = b.message_by_topic('/applehand/finger3/imu')
        data_f3_joint = b.message_by_topic('/applehand/finger3/jointstate')
        data_trial_events = b.message_by_topic('/apple_trial_events')
