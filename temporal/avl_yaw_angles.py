import random, math, csv
import numpy as np
import matplotlib.pyplot as plt

# ---------------------------------------- Step 6 - Read the csv file ----------------------------------------------
location = '/home/avl/PycharmProjects/AppleProxy/'
# Read the csv file with all the angles from the real-apple picks
# [Hand-Stem, Stem-Gravity, Hand-gravity]

file = 'real_picks_angles.csv'
with open(location + file, 'r') as f:
    reader = csv.reader(f)
    angles = list(reader)
apples = len(angles)

# Add index into each pick
i = 1
for pick in angles:
    pick.append(i)
    # print(pick)
    i += 1

# Sort the list according to the Stem-Gravity angle in order to simplify the proxy arrangement
real_pick_angles = angles
# real_pick_angles = np.array(angles)
# real_pick_angles = real_pick_angles[np.argsort(real_pick_angles[:, 1])]

for j in range(len(real_pick_angles)):
    pick = real_pick_angles[j]
    # Read the angles individually, convert to float, remove decimals
    hand_stem_angle = round(float(pick[0]), 0)
    stem_gravity_angle = round(float(pick[1]), 0)
    hand_gravity_angle = round(float(pick[2]), 0)
    index = round(float(pick[3]), 0)

    print("\nLoop # ", j)
    print("Replicating pick")
    print("Pick %i, Hand-Stem: %.0f, Stem-Gravity: %.0f, Hand-Gravity: %.0f" % (index,
    hand_stem_angle, stem_gravity_angle, hand_gravity_angle))

    # Find the Yaw angle
    stem_gravity = math.radians(stem_gravity_angle)
    hand_gravity = math.radians(hand_gravity_angle)
    hand_stem = math.radians(hand_stem_angle)

    stem_vector = np.array([0, np.sin(stem_gravity), -np.cos(stem_gravity)])
    hand_z = -np.cos(hand_gravity)
    hand_radius = np.sin(hand_gravity)
    for i in range(360):
        hand_x = hand_radius * np.cos(np.radians(i))
        hand_y = hand_radius * np.sin(np.radians(i))
        hand_prelim_vector = np.array([hand_x, hand_y, hand_z])
        hand_stem_prelim = np.arccos(np.dot(hand_prelim_vector, stem_vector))

        if abs(hand_stem_prelim - hand_stem) < 0.005:
            hand_vector = hand_prelim_vector
            yaw = i

    real_pick_angles[j].append(yaw)


with open(location + 'real_picks_angles_yaw.csv', 'w') as f:
    write = csv.writer(f)
    write.writerows(real_pick_angles)