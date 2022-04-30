"""
Created by: velasale@oregonstate.edu
"""
import bagpy
import pandas as pd
from bagpy import bagreader
import matplotlib.pyplot as plt
import matplotlib
# matplotlib.use('Qt5Agg')
import math
import numpy as np
import seaborn as sns
import csv
import os
from scipy.ndimage.filters import gaussian_filter

# sns.set()  # Setting seaborn as default style even if use only matplotlib
plt.close('all')


def net_value(var_x, var_y, var_z):
    """
    Obtain the net value of a vector, given the 3 components
    :param var_x:
    :param var_y:
    :param var_z:
    :return:
    """
    net = [None] * len(var_x)
    for i in range(len(var_x)):
        net[i] = math.sqrt(var_x[i] ** 2 + var_y[i] ** 2 + var_z[i] ** 2)
    return net


def elapsed_time(variable, time_stamp):
    """
    Simplifies the time axis, by subtracting the initial time
    :param variable: Reference variable to obtain the size of the time array
    :param time_stamp: The time stamp array that is going to be simplified
    :return: Simplified time as Elapsed Time
    """
    elapsed = [None] * len(variable)
    for i in range(len(variable)):
        elapsed[i] = time_stamp[i] - time_stamp[0]
    return elapsed


def find_instance(array, time_array, threshold, initial_time, case='starts'):
    """
    There are also some events that are important to spot such as when the fingers start moving indeed
    Since these are not published by the hand, we'll find those instances by finding the point at which the slope of any
    of the joints changes
    :param array:
    :param time_array:
    :param rate: Rate that we are looking for
    :param initial_time:
    :param case: To make this function more general 'starts' is when movement starts, or 'stops' when it stops
    :return: Time at which the variable starts considerably changing
    """
    # Step 1: Find the index of the initial time to start looking
    global i
    for initial_time_index in range(len(time_array)):
        if time_array[initial_time_index] > initial_time:
            break

    # try:
    #     for i in range(initial_time_index, len(array)):
    #         derivative = (array[i + 1] - array[i]) / (time_array[i + 1] - time_array[i])
    #         # print('the derivative is: ', derivative)
    #         if abs(derivative) < rate and case == 'stops':
    #             # print('the derivative is: ', derivative)
    #             # print('The time at which is starts changing is: ', time_array[i])
    #             return i, time_array[i]
    #             break
    #         if abs(derivative) > rate and case == 'starts':
    #             return i, time_array[i]
    #             break

    try:
        for i in range(initial_time_index, len(array)):
            if abs(array[i]) > threshold and case == 'starts':
                break
            elif abs(array[i]) < threshold and case == 'stops':
                break

        return i, time_array[i]

    except KeyError or TypeError:
        if case == 'starts':
            return 1e6, 1e6  # just big numbers
        else:
            return 0, 0  # low numbers


def plot_features(plt, a, b, c, d, e, f, g, h, x_min, x_max, title, label, sigma):
    """
    Add some common features into all the plots in order to ease their analysis, such as shaded areas and dashed lines
    :param x_max:
    :param x_min:
    :param plt:
    :param a: Time of the Open Hand Event
    :param b: Time of the Close Hand Event
    :param c: Time of the Pull Event
    :param d: Time of the Open Hand Event at the end
    :param e: Time of the Servos Start
    :param f: Time of the Servos Stop
    :param g: Time of the UR5 Start
    :param h: Time of the UR5 Stop
    :return:
    """
    plt.legend()
    plt.xlim(x_min, x_max)
    xmin, xmax, ymin, ymax = plt.axis()
    plt.axvspan(a, b, color='y', alpha=0.3, lw=0)
    plt.annotate('Open hand', (a, 0.95 * ymin))
    plt.axvspan(b, c, color='b', alpha=0.3, lw=0)
    plt.annotate('Close hand', (b, 0.95 * ymin))
    plt.axvspan(c, d, color='r', alpha=0.3, lw=0)
    plt.annotate('Pull', (c, 0.95 * ymin))
    plt.axvspan(d, xmax, color='g', alpha=0.3, lw=0)
    plt.annotate('Open hand', (d, 0.95 * ymin))
    plt.axvline(x=e, color='k', linestyle='dashed')
    plt.annotate('F1 Servo STARTS moving', (e, 0.85 * ymin))
    plt.axvline(x=f, color='k', linestyle=(0, (5, 10)))
    plt.annotate('F1 Servo STOPS moving', (f, 0.80 * ymin))
    plt.axvline(x=g, color='k', linestyle='dotted')
    plt.annotate('UR5e STARTS moving', (g, 0.85 * ymin))
    plt.axvline(x=h, color='k', linestyle=(0, (1, 10)))
    plt.annotate('UR5e STOPS moving', (h, 0.80 * ymin))
    plt.annotate('sigma = ' + str(sigma), (xmin, ymax))
    plt.xlabel('Elapsed time [sec]')
    plt.title(title + ' ' + f'$\\bf{label}$')
    plt.savefig('plots/' + title + '.png')


def broken_axes(time, variables, legends, e, f, g, h, title, y_label, label):
    """
    Creates a plot with two subplots and a break point in the x-axis
    Reference: https://stackoverflow.com/questions/32185411/break-in-x-axis-of-matplotlib
    :param y_label:
    :param label: Label of the apple pick: Successful or Failure
    :param time:
    :param variables:
    :param legends:
    :param e:
    :param f:
    :param g:
    :param h:
    :return:
    """

    fig, (ax, ax2) = plt.subplots(1, 2, sharey=True, facecolor='w', figsize=(16, 9))

    for i in range(len(variables)):
        ax.plot(time, variables[i], label=legends[i])
        ax2.plot(time, variables[i], label=legends[i])

    ax.legend()
    ax2.legend()
    ax.set_title('Grasp')
    ax2.set_title('Pick')
    ax.grid()
    ax2.grid()
    ax.set_xlim(e - 0.5, f + 0.5)
    ax2.set_xlim(g - 0.5, h + 0.5)
    ax.spines['right'].set_visible(False)
    ax2.spines['left'].set_visible(False)

    ax.set_ylabel(y_label)
    ax.yaxis.tick_left()
    # ax.tick_params(labelright='off')
    ax2.tick_params(labelleft='off')
    ax2.yaxis.tick_right()

    d = .015
    kwargs = dict(transform=ax.transAxes, color='k', clip_on=False)
    ax.plot((1 - d, 1 + d), (-d, +d), **kwargs)
    ax.plot((1 - d, 1 + d), (1 - d, 1 + d), **kwargs)
    kwargs.update(transform=ax2.transAxes)
    ax2.plot((-d, +d), (1 - d, 1 + d), **kwargs)
    ax2.plot((-d, +d), (-d, +d), **kwargs)

    plt.xlabel('Elapsed time [sec]')
    plt.suptitle(title + ' ' + f'$\\bf{label}$')
    plt.savefig('plots/' + title + ' ' + label + '.png')


def look_at_labels(location, prefix):
    """
    Looks for the csv file of the bagfile, in order to read the labels from the metadata
    :param location:
    :param prefix:
    :param label:
    :return:
    """
    # --- Step 1: Look for the file
    for filename in os.listdir(location):
        if filename.startswith(prefix + '_'):
            print(filename)  # print the name of the file to make sure it is what
            break

    # --- Open the file
    with open(location + filename) as f:
        reader = csv.reader(f)
        data = list(reader)
        print('Result of Pick was: ', data[1][10])

    # --- Improve the label
    if data[1][10] == 's':
        result = '(Successful-Pick)'
    elif data[1][10] == 'f':
        result = '(Failed-Pick)'
    else:
        result = 'heads up... something is wrong'

    return result


def event_times(trial_events_elapsed_time, event_indexes, f1, f2, f3, arm):
    """
    Finds the times at which the hand's servos and arms motors start and stop moving.
    These instants are importante because they correspond to the preiods when the Grasping and Pick happen.
    Therefore, we would focus on the data within these values, and disregard the rest.
    :param trial_events_elapsed_time:
    :param event_indexes:
    :param f1: Finger 1 time and speed
    :param f2: Finger 2 time and speed
    :param f3: Finger 3 time and speed
    :param arm: Arm Joints time and speeds
    :return:
    """
    # Initial Open Hand Event
    open_hand_event_index = event_indexes[1]
    open_hand_event_time = trial_events_elapsed_time[open_hand_event_index]

    f1_state_elapsed_time = f1[0]
    f1_state_speed = f1[1]
    f2_state_elapsed_time = f2[0]
    f2_state_speed = f2[1]
    f3_state_elapsed_time = f3[0]
    f3_state_speed = f3[1]

    arm_joints_elapsed_time = arm[0]
    joint_0_spd = arm[1]
    joint_1_spd = arm[2]
    joint_2_spd = arm[3]
    joint_3_spd = arm[4]
    joint_4_spd = arm[5]
    joint_5_spd = arm[6]

    if len(event_indexes) == 2:
        # This was the case of real_apple_pick11
        pulling_apple_event_index = event_indexes[1]
        final_open_hand_event_index = event_indexes[1]
        closing_hand_event_index = event_indexes[1]

    elif len(event_indexes) == 4:
        # This was the case of the real_apple_pick 1 to 10
        pulling_apple_event_index = event_indexes[3]
        final_open_hand_event_index = event_indexes[3]
        closing_hand_event_index = event_indexes[2]

    elif len(event_indexes) == 5:
        # This was the case of the real_apple_pick 12 to 33
        pulling_apple_event_index = event_indexes[3]
        final_open_hand_event_index = event_indexes[4]
        closing_hand_event_index = event_indexes[2]

    elif len(event_indexes) == 6:
        # This was the case of the real_apple_pick 34 to 77
        pulling_apple_event_index = event_indexes[3]
        final_open_hand_event_index = event_indexes[5]
        closing_hand_event_index = event_indexes[2]

    # Be careful when moving from ROS events into hand's, because they don't have the same indexes
    pulling_apple_event_time = trial_events_elapsed_time[pulling_apple_event_index]
    final_open_hand_event_time = trial_events_elapsed_time[final_open_hand_event_index]
    closing_hand_event_time = trial_events_elapsed_time[closing_hand_event_index]

    a = open_hand_event_time
    b = closing_hand_event_time
    c = pulling_apple_event_time
    d = final_open_hand_event_time

    # Servos Start Moving Event
    # Find the instance when the fingers' motors start moving (index and value)
    print('Point to start evaluating: ', closing_hand_event_time)
    i1, e1 = find_instance(f1_state_speed, f1_state_elapsed_time, 0.01, closing_hand_event_time, 'starts')
    i2, e2 = find_instance(f2_state_speed, f2_state_elapsed_time, 0.01, closing_hand_event_time, 'starts')
    i3, e3 = find_instance(f3_state_speed, f3_state_elapsed_time, 0.01, closing_hand_event_time, 'starts')
    e = min(e1, e2, e3)
    print('\nFinger servos start moving at: %.2f, %.2f and %.2f ' % (e1, e2, e3))
    print('The time delay between event and servo moving is: %.2f' % (e - b))

    # Servos Stop Moving Event
    # Find the instance when the finger's motors stop indeed moving
    j1, f1 = find_instance(f1_state_speed, f1_state_elapsed_time, 0.01, e, 'stops')
    j2, f2 = find_instance(f2_state_speed, f2_state_elapsed_time, 0.01, e, 'stops')
    j3, f3 = find_instance(f3_state_speed, f3_state_elapsed_time, 0.01, e, 'stops')
    f = max(f1, f2, f3)
    print('Finger servos stop moving at: %.2f, %.2f and %.2f' % (f1, f2, f3))

    if len(event_indexes) == 4:
        c = f

    k0, g0 = find_instance(joint_0_spd, arm_joints_elapsed_time, 0.01, c, 'starts')
    k1, g1 = find_instance(joint_1_spd, arm_joints_elapsed_time, 0.01, c, 'starts')
    k2, g2 = find_instance(joint_2_spd, arm_joints_elapsed_time, 0.01, c, 'starts')
    k3, g3 = find_instance(joint_3_spd, arm_joints_elapsed_time, 0.01, c, 'starts')
    k4, g4 = find_instance(joint_4_spd, arm_joints_elapsed_time, 0.01, c, 'starts')
    k5, g5 = find_instance(joint_5_spd, arm_joints_elapsed_time, 0.01, c, 'starts')
    g = min(g0, g1, g2, g3, g4, g5)
    print(
        "The times at which the UR5 joints start are: %.2f, %.2f, %2.f, %2.f, %.2f and %.2f" % (g0, g1, g2, g3, g4, g5))
    print('\nUR5 starts moving at: %.2f ' % g)

    if len(event_indexes) == 4:
        c = g

    k = max(g0, g1, g2, g3, g4, g5)
    # print("The values of k are: %.2f, %.2f, %2.f, %2.f, %.2f and %.2f" % (k0, k1, k2, k3, k4, k5))

    # Arm Stops pulling apple
    l0, h0 = find_instance(joint_0_spd, arm_joints_elapsed_time, 0.001, g, 'stops')
    l1, h1 = find_instance(joint_1_spd, arm_joints_elapsed_time, 0.001, g, 'stops')
    l2, h2 = find_instance(joint_2_spd, arm_joints_elapsed_time, 0.001, g, 'stops')
    l3, h3 = find_instance(joint_3_spd, arm_joints_elapsed_time, 0.001, g, 'stops')
    l4, h4 = find_instance(joint_4_spd, arm_joints_elapsed_time, 0.001, g, 'stops')
    l5, h5 = find_instance(joint_5_spd, arm_joints_elapsed_time, 0.001, g, 'stops')
    h = max(h0, h1, h2, h3, h4, h5)
    print(
        "The times at which the UR5 joints stop are: %.2f, %.2f, %2.f, %2.f, %.2f and %.2f" % (h0, h1, h2, h3, h4, h5))
    print('UR5 stops moving at: %.2f' % h)

    return a, b, c, d, e, f, g, h


def generate_plots(i, zoom='all', sigma=0):
    """
    This is the main function that generates all the plots from a bag file, given the number 'i' of the pick.
    :param sigma: Plot filter parameter
    :param zoom: Type of Zoom wanted for the plots... (all, both, grasp or pick)
    :param i: Number of the apple pick
    :return:
    """
    # --------------------------------- Step 1: Search the folder for the bagfiles -----------------------------------------
    # Hard Drive
    # location = '/media/avl/StudyData/ApplePicking Data/5 - Real Apple with Hand Closing Fixed/bagfiles'

    # Lab's Laptop
    # location = '/home/avl/PycharmProjects/icra22/bagfiles'

    # Lab's PC
    # location = '/home/avl/ur_ws/src/apple_proxy/bag_files'
    location = '/home/avl/PycharmProjects/AppleProxy/bagfiles/'

    # Alejo's laptop
    # location = '/home/avl/PycharmProjects/appleProxy/bagfiles/'

    number = str(i)
    # file = 'apple_proxy_pick' + number
    file = 'fall21_real_apple_pick' + number
    b = bagreader(location + '/' + file + '.bag')

    # Look at the metadata file
    label = look_at_labels(location, file)

    # Get the list of topics available in the file
    print(b.topic_table)

    # Read each topic
    # ARM's topics
    # a. Arm's Wrench topic: forces and torques
    data_arm_wrench = b.message_by_topic('wrench')
    arm_wrench = pd.read_csv(data_arm_wrench)

    # b. Arm's Joint_States topic: angular positions of the 6 joints
    data_arm_joints = b.message_by_topic('joint_states')
    arm_joints = pd.read_csv(data_arm_joints)

    # HAND's topics
    # c. Hand's finger1 imu
    data_f1_imu = b.message_by_topic('/applehand/finger1/imu')
    f1_imu = pd.read_csv(data_f1_imu)

    # d. Hand's finger1 joint state
    data_f1_joint = b.message_by_topic('/applehand/finger1/jointstate')
    f1_state = pd.read_csv(data_f1_joint)

    # e. Hand's finger2 imu
    data_f2_imu = b.message_by_topic('/applehand/finger2/imu')
    f2_imu = pd.read_csv(data_f2_imu)

    # e. Hand's finger2 joint state
    data_f2_joint = b.message_by_topic('/applehand/finger2/jointstate')
    f2_state = pd.read_csv(data_f2_joint)

    # e. Hand's finger3 imu
    data_f3_imu = b.message_by_topic('/applehand/finger3/imu')
    f3_imu = pd.read_csv(data_f3_imu)

    # f. Hand's finger3 joint state
    data_f3_joint = b.message_by_topic('/applehand/finger3/jointstate')
    f3_state = pd.read_csv(data_f3_joint)

    # EVENT's topic
    # g. Trial events
    data_trial_events = b.message_by_topic('/apple_trial_events')
    trial_events = pd.read_csv(data_trial_events)

    # --------------------------  Step 2: Extract each vector from the csv's, and adjust the time---------------------------
    # TIME STAMPS
    arm_time_stamp = arm_wrench.iloc[:, 0]
    arm_joints_time_stamp = arm_joints.iloc[:, 0]
    f1_imu_time_stamp = f1_imu.iloc[:, 0]
    f1_state_time_stamp = f1_state.iloc[:, 0]
    f2_imu_time_stamp = f2_imu.iloc[:, 0]
    f2_state_time_stamp = f2_state.iloc[:, 0]
    f3_imu_time_stamp = f3_imu.iloc[:, 0]
    f3_state_time_stamp = f3_state.iloc[:, 0]
    trial_events_time_stamp = trial_events.iloc[:, 0]

    # ARM
    forces_x = arm_wrench.iloc[:, 5]
    forces_y = arm_wrench.iloc[:, 6]
    forces_z = arm_wrench.iloc[:, 7]
    torques_x = arm_wrench.iloc[:, 8]
    torques_y = arm_wrench.iloc[:, 9]
    torques_z = arm_wrench.iloc[:, 10]

    joint_0_pos = arm_joints.iloc[:, 6]
    joint_1_pos = arm_joints.iloc[:, 7]
    joint_2_pos = arm_joints.iloc[:, 8]
    joint_3_pos = arm_joints.iloc[:, 9]
    joint_4_pos = arm_joints.iloc[:, 10]
    joint_5_pos = arm_joints.iloc[:, 11]

    joint_0_spd = arm_joints.iloc[:, 12]
    joint_1_spd = arm_joints.iloc[:, 13]
    joint_2_spd = arm_joints.iloc[:, 14]
    joint_3_spd = arm_joints.iloc[:, 15]
    joint_4_spd = arm_joints.iloc[:, 16]
    joint_5_spd = arm_joints.iloc[:, 17]

    # HAND
    f1_state_position = f1_state.iloc[:, 5]
    f1_state_speed = f1_state.iloc[:, 6]
    f1_state_effort = f1_state.iloc[:, 7]

    f2_state_position = f2_state.iloc[:, 5]
    f2_state_speed = f2_state.iloc[:, 6]
    f2_state_effort = f2_state.iloc[:, 7]

    f3_state_position = f3_state.iloc[:, 5]
    f3_state_speed = f3_state.iloc[:, 6]
    f3_state_effort = f3_state.iloc[:, 7]

    f1_acc_x = f1_imu.iloc[:, 5]
    f1_acc_y = f1_imu.iloc[:, 6]
    f1_acc_z = f1_imu.iloc[:, 7]

    f2_acc_x = f2_imu.iloc[:, 5]
    f2_acc_y = f2_imu.iloc[:, 6]
    f2_acc_z = f2_imu.iloc[:, 7]

    f3_acc_x = f3_imu.iloc[:, 5]
    f3_acc_y = f3_imu.iloc[:, 6]
    f3_acc_z = f3_imu.iloc[:, 7]

    f1_gyro_x = f1_imu.iloc[:, 8]
    f1_gyro_y = f1_imu.iloc[:, 9]
    f1_gyro_z = f1_imu.iloc[:, 10]

    f2_gyro_x = f2_imu.iloc[:, 8]
    f2_gyro_y = f2_imu.iloc[:, 9]
    f2_gyro_z = f2_imu.iloc[:, 10]

    f3_gyro_x = f3_imu.iloc[:, 8]
    f3_gyro_y = f3_imu.iloc[:, 9]
    f3_gyro_z = f3_imu.iloc[:, 10]

    arm_elapsed_time = elapsed_time(forces_x, arm_time_stamp)
    arm_joints_elapsed_time = elapsed_time(arm_joints, arm_joints_time_stamp)
    f1_imu_elapsed_time = elapsed_time(f1_imu, f1_imu_time_stamp)
    f1_state_elapsed_time = elapsed_time(f1_state, f1_state_time_stamp)
    f2_imu_elapsed_time = elapsed_time(f2_imu, f2_imu_time_stamp)
    f2_state_elapsed_time = elapsed_time(f2_state, f2_state_time_stamp)
    f3_imu_elapsed_time = elapsed_time(f3_imu, f3_imu_time_stamp)
    f3_state_elapsed_time = elapsed_time(f3_state, f3_state_time_stamp)
    trial_events_elapsed_time = elapsed_time(trial_events, trial_events_time_stamp)

    # ----------------------------- Step 3: Get the Events' times ------------------------------------------------------
    # First get the indexes when the events happen
    event_indexes = np.where(np.diff(trial_events.iloc[:, 1], prepend=np.nan))[0]
    print('The events indexes are: ', event_indexes)

    a, b, c, d, e, f, g, h = event_times(trial_events_elapsed_time,
                                         event_indexes,
                                         [f1_state_elapsed_time, f1_state_speed],
                                         [f2_state_elapsed_time, f2_state_speed],
                                         [f3_state_elapsed_time, f3_state_speed],
                                         [arm_joints_elapsed_time, joint_0_spd, joint_1_spd, joint_2_spd, joint_3_spd,
                                          joint_4_spd, joint_5_spd])

    # From here...
    # ... this is where all the code of function event_times was taken from
    # Until here..

    x_min = min(arm_time_stamp)
    x_max = max(arm_time_stamp)

    if zoom == 'all':
        x_min = 0
        x_max = max(arm_elapsed_time)
    elif zoom == 'grasp':
        x_min = e - 0.8
        x_max = f + 1
    elif zoom == 'pick':
        x_min = g - 1
        x_max = h + 1

    # -----------------------------  Step 4: Smooth data a little bit --------------------------------------------------
    # Smooth data a little bit
    # Sigma depends on how noisy your data is, play with it!

    forces_x = gaussian_filter(forces_x, sigma)
    forces_y = gaussian_filter(forces_y, sigma)
    forces_z = gaussian_filter(forces_z, sigma)
    net_force = net_value(forces_x, forces_y, forces_z)

    torques_x = gaussian_filter(torques_x, sigma)
    torques_y = gaussian_filter(torques_y, sigma)
    torques_z = gaussian_filter(torques_z, sigma)
    net_torque = net_value(torques_x, torques_y, torques_z)

    f1_acc_x = gaussian_filter(f1_acc_x, sigma)
    f1_acc_y = gaussian_filter(f1_acc_y, sigma)
    f1_acc_z = gaussian_filter(f1_acc_z, sigma)
    net_f1_acc = net_value(f1_acc_x, f1_acc_y, f1_acc_z)

    f2_acc_x = gaussian_filter(f2_acc_x, sigma)
    f2_acc_y = gaussian_filter(f2_acc_y, sigma)
    f2_acc_z = gaussian_filter(f2_acc_z, sigma)
    net_f2_acc = net_value(f2_acc_x, f2_acc_y, f2_acc_z)

    f3_acc_x = gaussian_filter(f3_acc_x, sigma)
    f3_acc_y = gaussian_filter(f3_acc_y, sigma)
    f3_acc_z = gaussian_filter(f3_acc_z, sigma)
    net_f3_acc = net_value(f3_acc_x, f3_acc_y, f3_acc_z)

    f1_gyro_x = gaussian_filter(f1_gyro_x, sigma)
    f1_gyro_y = gaussian_filter(f1_gyro_y, sigma)
    f1_gyro_z = gaussian_filter(f1_gyro_z, sigma)

    f2_gyro_x = gaussian_filter(f2_gyro_x, sigma)
    f2_gyro_y = gaussian_filter(f2_gyro_y, sigma)
    f2_gyro_z = gaussian_filter(f2_gyro_z, sigma)

    f3_gyro_x = gaussian_filter(f3_gyro_x, sigma)
    f3_gyro_y = gaussian_filter(f3_gyro_y, sigma)
    f3_gyro_z = gaussian_filter(f3_gyro_z, sigma)

    # ------------------------------------  Step 5: Plot Results -----------------------------------------------------------
    #
    # First select the kind of x-axis time that you want: (a) Original Time Stamps (b) human-readable time stamp
    # (a) Original Time Stamps
    # arm_time_ref = arm_time_stamp
    # arm_joints_time_ref = arm_joints_time_stamp
    # f1_state_time_ref = f1_state_time_stamp
    # f1_imu_time_ref = f1_imu_time_stamp
    # f2_state_time_ref = f2_state_time_stamp
    # f2_imu_time_ref = f2_imu_time_stamp
    # f3_state_time_ref = f3_state_time_stamp
    # f3_imu_time_ref = f3_imu_time_stamp
    # trial_events_time_ref = trial_events_time_stamp

    # (b) Human Readable Time Stamps
    arm_time_ref = arm_elapsed_time
    arm_joints_time_ref = arm_joints_elapsed_time
    f1_state_time_ref = f1_state_elapsed_time
    f1_imu_time_ref = f1_imu_elapsed_time
    f2_state_time_ref = f2_state_elapsed_time
    f2_imu_time_ref = f2_imu_elapsed_time
    f3_state_time_ref = f3_state_elapsed_time
    f3_imu_time_ref = f3_imu_elapsed_time
    trial_events_time_ref = trial_events_elapsed_time

    # ARM figures
    # Arm's Forces
    # plt.figure(figsize=(16, 9))
    # # baxes = brokenaxes(xlims=((19, 22), (38, 43)), hspace=0.5)
    # plt.plot(arm_time_ref, forces_x, color='green', label='Force_x')
    # plt.plot(arm_time_ref, forces_y, color='blue', label='Force_y')
    # plt.plot(arm_time_ref, forces_z, color='red', label='Force_z')
    # plt.plot(arm_time_ref, net_force, color='k', label='Net Force')
    # plt.plot(trial_events_time_ref, trial_events.iloc[:, 1], color='m', label='event')
    title = file + ' - UR5e - (FT) Wrist Forces'
    # plt.ylabel('Force [N]')
    # plot_features(plt, a, b, c, d, e, f, g, h, x_min, x_max, title, label, sigma)
    broken_axes(arm_time_ref,
                [forces_x, forces_y, forces_z, net_force],
                ['forces_x', 'forces_y', 'forces_z', 'net_force'],
                e, f, g, h, title, 'Force [N]', label)

    #
    # # Arm's Torques
    # plt.figure(figsize=(16, 9))
    # plt.plot(arm_time_ref, torques_x, color='green', label='Torque_x')
    # plt.plot(arm_time_ref, torques_y, color='blue', label='Torque_y')
    # plt.plot(arm_time_ref, torques_z, color='red', label='Torque_z')
    # plt.plot(arm_time_ref, net_torque, color='k', label='Net Torque')
    title = file + ' - UR5e - (FT) Wrist Torques'
    # plt.ylabel('Torque [Nm]')
    # plot_features(plt, a, b, c, d, e, f, g, h, x_min, x_max, title, label, sigma)
    broken_axes(arm_time_ref,
                [torques_x, torques_y, torques_z, net_torque],
                ['torques_x', 'torques_y', 'torques_z', 'net_torque'],
                e, f, g, h, title, 'Torque [Nm]', label)

    #
    # # Arm's Joints
    # plt.figure(figsize=(16, 9))
    # plt.plot(arm_joints_time_ref, joint_0_pos, color='green', label='Joint 0')
    # plt.plot(arm_joints_time_ref, joint_1_pos, color='blue', label='Joint 1')
    # plt.plot(arm_joints_time_ref, joint_2_pos, color='red', label='Joint 2')
    # plt.plot(arm_joints_time_ref, joint_3_pos, color='green', label='Wrist 1')
    # plt.plot(arm_joints_time_ref, joint_4_pos, color='blue', label='Wrist 2')
    # plt.plot(arm_joints_time_ref, joint_5_pos, color='red', label='Wrist 3')
    title = file + ' - UR5e - Joint Angular Positions'
    # plt.ylabel('Angular Positions [rad]')
    # plot_features(plt, a, b, c, d, e, f, g, h, x_min, x_max, title, label, sigma)
    broken_axes(arm_joints_time_ref,
                [joint_0_pos, joint_1_pos, joint_2_pos, joint_3_pos, joint_4_pos, joint_5_pos],
                ['joint_0_pos', 'joint_1_pos', 'joint_2_pos', 'joint_3_pos', 'joint_4_pos', 'joint_5_pos'],
                e, f, g, h, title, 'Angular Positions [rad]', label)

    # # Finger 1's Joint States
    # fig = plt.figure(figsize=(16, 9))
    # plt.plot(f1_state_time_ref, f1_state_position, color='r', label='Position')
    # plt.plot(f1_state_time_ref, f1_state_speed, color='g', label='Speed')
    # plt.plot(f1_state_time_ref, f1_state_effort, color='b', label='Effort')
    title = file + ' - Finger 1 - Joint States'
    # plt.ylabel('f1 - States')
    # plot_features(plt, a, b, c, d, e, f, g, h, x_min, x_max, title, label, sigma)
    broken_axes(f1_state_time_ref,
                [f1_state_position, f1_state_speed, f1_state_effort],
                ['f1_state_position', 'f1_state_speed', 'f1_state_effort'],
                e, f, g, h, title, 'f1 - States', label)

    #
    # # Finger 1's accelerometers
    # fig = plt.figure(figsize=(16, 9))
    # plt.plot(f1_imu_time_ref, f1_acc_x, color='r', label='acc x')
    # plt.plot(f1_imu_time_ref, f1_acc_y, color='g', label='acc y')
    # plt.plot(f1_imu_time_ref, f1_acc_z, color='b', label='acc z')
    # plt.plot(f1_imu_time_ref, net_f1_acc, color='k', label='Net F1 acc')
    title = file + ' - Finger 1 - (IMU) Linear Acceleration'
    # plt.ylabel('Linear Acceleration [g]')
    # plot_features(plt, a, b, c, d, e, f, g, h, x_min, x_max, title, label, sigma)
    broken_axes(f1_imu_time_ref,
                [f1_acc_x, f1_acc_y, f1_acc_z, net_f1_acc],
                ['f1_acc_x', 'f1_acc_y', 'f1_acc_z', 'net_f1_acc'],
                e, f, g, h, title, 'Linear Acceleration [g]', label)

    #
    # # Finger 1's Gyroscopes
    # fig = plt.figure(figsize=(16, 9))
    # plt.plot(f1_imu_time_ref, f1_gyro_x, color='r', label='gyro x')
    # plt.plot(f1_imu_time_ref, f1_gyro_y, color='g', label='gyro y')
    # plt.plot(f1_imu_time_ref, f1_gyro_z, color='b', label='gyro z')
    title = file + ' - Finger 1 - (IMU) Angular Velocity'
    # plt.ylabel('Angular Velocity [deg/s]')
    # plot_features(plt, a, b, c, d, e, f, g, h, x_min, x_max, title, label, sigma)
    broken_axes(f1_imu_time_ref,
                [f1_gyro_x, f1_gyro_y, f1_gyro_z],
                ['f1_gyro_x', 'f1_gyro_y', 'f1_gyro_z'],
                e, f, g, h, title, 'Angular Velocity [deg/s]', label)

    #
    # # Finger 2's Joint States
    # fig = plt.figure(figsize=(16, 9))
    # plt.plot(f2_state_time_ref, f2_state_position, color='r', label='Position')
    # plt.plot(f2_state_time_ref, f2_state_speed, color='g', label='Speed')
    # plt.plot(f2_state_time_ref, f2_state_effort, color='b', label='Effort')
    title = file + ' - Finger 2 - Joint States'
    # plt.ylabel('f2 - States')
    # plot_features(plt, a, b, c, d, e, f, g, h, x_min, x_max, title, label, sigma)
    broken_axes(f2_state_time_ref,
                [f2_state_position, f2_state_speed, f2_state_effort],
                ['f2_state_position', 'f2_state_speed', 'f2_state_effort'],
                e, f, g, h, title, 'f2 - States', label)
    #
    # # Finger 2;s Accelerometers
    # fig = plt.figure(figsize=(16, 9))
    # plt.plot(f2_imu_time_ref, f2_acc_x, color='r', label='acc x')
    # plt.plot(f2_imu_time_ref, f2_acc_y, color='g', label='acc y')
    # plt.plot(f2_imu_time_ref, f2_acc_z, color='b', label='acc z')
    # plt.plot(f2_imu_time_ref, net_f2_acc, color='k', label='Net F2 acc')
    title = file + ' - Finger 2 - (IMU) Linear Acceleration'
    # plt.ylabel('Linear Acceleration [g]')
    # plot_features(plt, a, b, c, d, e, f, g, h, x_min, x_max, title, label, sigma)
    broken_axes(f2_imu_time_ref,
                [f2_acc_x, f2_acc_y, f2_acc_z, net_f2_acc],
                ['f2_acc_x', 'f2_acc_y', 'f2_acc_z', 'net_f2_acc'],
                e, f, g, h, title, 'Linear Acceleration [g]', label)

    #
    # # Finger 2's Gyroscopes
    # fig = plt.figure(figsize=(16, 9))
    # plt.plot(f2_imu_time_ref, f2_gyro_x, color='r', label='gyro x')
    # plt.plot(f2_imu_time_ref, f2_gyro_y, color='g', label='gyro y')
    # plt.plot(f2_imu_time_ref, f2_gyro_z, color='b', label='gyro z')
    title = file + ' - Finger 2 - (IMU) Angular Velocity'
    # plt.ylabel('Angular Velocity [deg/s]')
    # plot_features(plt, a, b, c, d, e, f, g, h, x_min, x_max, title, label, sigma)
    broken_axes(f2_imu_time_ref,
                [f2_gyro_x, f2_gyro_y, f2_gyro_z],
                ['f2_gyro_x', 'f2_gyro_y', 'f2_gyro_z'],
                e, f, g, h, title, 'Angular Velocity [deg/s]', label)

    #
    # # Finger 3's Joint States
    # fig = plt.figure(figsize=(16, 9))
    # plt.plot(f3_state_time_ref, f3_state_position, color='r', label='Position')
    # plt.plot(f3_state_time_ref, f3_state_speed, color='g', label='Speed')
    # plt.plot(f3_state_time_ref, f3_state_effort, color='b', label='Effort')
    title = file + ' - Finger 3 - Joint States'
    # plt.ylabel('f3 - States')
    # plot_features(plt, a, b, c, d, e, f, g, h, x_min, x_max, title, label, sigma)
    broken_axes(f3_state_time_ref,
                [f3_state_position, f3_state_speed, f3_state_effort],
                ['f3_state_position', 'f3_state_speed', 'f3_state_effort'],
                e, f, g, h, title, 'f3 - States', label)

    #
    # # Finger 3's Accelerometers
    # fig = plt.figure(figsize=(16, 9))
    # plt.plot(f3_imu_time_ref, f3_acc_x, color='r', label='acc x')
    # plt.plot(f3_imu_time_ref, f3_acc_y, color='g', label='acc y')
    # plt.plot(f3_imu_time_ref, f3_acc_z, color='b', label='acc z')
    # plt.plot(f3_imu_time_ref, net_f3_acc, color='k', label='Net F3 acc')
    title = file + ' - Finger 3 - (IMU) Linear Acceleration'
    # plt.ylabel('Linear Acceleration [g]')
    # plot_features(plt, a, b, c, d, e, f, g, h, x_min, x_max, title, label, sigma)
    broken_axes(f3_imu_time_ref,
                [f3_acc_x, f3_acc_y, f3_acc_z, net_f3_acc],
                ['f3_acc_x', 'f3_acc_y', 'f3_acc_z', 'net_f3_acc'],
                e, f, g, h, title, 'Linear Acceleration [g]', label)
    #
    # # Finger 3's Gyroscopes
    # fig = plt.figure(figsize=(16, 9))
    # plt.plot(f3_imu_time_ref, f3_gyro_x, color='r', label='gyro x')
    # plt.plot(f3_imu_time_ref, f3_gyro_y, color='g', label='gyro y')
    # plt.plot(f3_imu_time_ref, f3_gyro_z, color='b', label='gyro z')
    title = file + ' - Finger 3 - (IMU) Angular Velocity'
    # plt.ylabel('Angular Velocity [deg/s]')
    # plot_features(plt, a, b, c, d, e, f, g, h, x_min, x_max, title, label, sigma)
    broken_axes(f3_imu_time_ref,
                [f3_gyro_x, f3_gyro_y, f3_gyro_z],
                ['f3_gyro_x', 'f3_gyro_y', 'f3_gyro_z'],
                e, f, g, h, title, 'Angular Velocity [deg/s]', label)

    # plt.show()


for i in range(12, 13):
    # Skip 11 because it had some issues
    if i == 11:
        continue

    print("Generating plots for pick No.: ", i)
    generate_plots(i, 'pick', 3)  # type 'all', 'grasp' or 'pick'
    plt.close('all')
