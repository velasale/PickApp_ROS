#!/usr/bin/env python
"""
This code is a tool to gather points in the real world, by using the UR5 as a probe,
and with those points calculate the location of the center of the apple and the orientation of the stem
with respect to the arm's base link
"""

import sympy as sym
import csv
import statistics as st

def approach_1(app_p1, app_p2, app_p3, app_p4, app_p5):
    """
    Obtain the center of a sphere / apple, and its radius, by using 4 points of the sphere
    The function takes the 5 combinations of 4 points, and averages them
    :param app_p1:
    :param app_p2:
    :param app_p3:
    :param app_p4:
    :param app_p5:
    :return:
    """
    # --- Step 2: Collect four points with the probe

    # Combination 1
    p1 = app_p1
    p2 = app_p2
    p3 = app_p3
    p4 = app_p4

    # --- Step 3: Elaborate the system of equations, in this case the equation of a sphere
    eq1 = sym.Eq((p1[0]-a)**2 + (p1[1]-b)**2 + (p1[2]-c)**2, r**2)
    eq2 = sym.Eq((p2[0]-a)**2 + (p2[1]-b)**2 + (p2[2]-c)**2, r**2)
    eq3 = sym.Eq((p3[0]-a)**2 + (p3[1]-b)**2 + (p3[2]-c)**2, r**2)
    eq4 = sym.Eq((p4[0]-a)**2 + (p4[1]-b)**2 + (p4[2]-c)**2, r**2)

    sln_1 = sym.solve([eq1, eq2, eq3, eq4], (a, b, c, r))
    # print('\n1st Combo sln')
    # print("The apple's center is located at (x, y, z): %.2f %.2f %.2f " % (sln_1[1][0], sln_1[1][1], sln_1[1][2]))
    # print("And it has a diameter of: %.3f cm" % (100 * 2 * sln_1[1][3]))

    # Combination 2
    p1 = app_p1
    p2 = app_p2
    p3 = app_p3
    p4 = app_p5

    # --- Step 3: Elaborate the system of equations, in this case the equation of a sphere
    eq1 = sym.Eq((p1[0]-a)**2 + (p1[1]-b)**2 + (p1[2]-c)**2, r**2)
    eq2 = sym.Eq((p2[0]-a)**2 + (p2[1]-b)**2 + (p2[2]-c)**2, r**2)
    eq3 = sym.Eq((p3[0]-a)**2 + (p3[1]-b)**2 + (p3[2]-c)**2, r**2)
    eq4 = sym.Eq((p4[0]-a)**2 + (p4[1]-b)**2 + (p4[2]-c)**2, r**2)

    sln_2 = sym.solve([eq1, eq2, eq3, eq4], (a, b, c, r))
    # print('\n2nd Combo sln')
    # print("The apple's center is located at (x, y, z): %.2f %.2f %.2f " % (sln_2[1][0], sln_2[1][1], sln_2[1][2]))
    # print("And it has a diameter of: %.3f cm" % (100 * 2 * sln_2[1][3]))


    # Combination 3
    p1 = app_p1
    p2 = app_p2
    p3 = app_p4
    p4 = app_p5

    # --- Step 3: Elaborate the system of equations, in this case the equation of a sphere
    eq1 = sym.Eq((p1[0]-a)**2 + (p1[1]-b)**2 + (p1[2]-c)**2, r**2)
    eq2 = sym.Eq((p2[0]-a)**2 + (p2[1]-b)**2 + (p2[2]-c)**2, r**2)
    eq3 = sym.Eq((p3[0]-a)**2 + (p3[1]-b)**2 + (p3[2]-c)**2, r**2)
    eq4 = sym.Eq((p4[0]-a)**2 + (p4[1]-b)**2 + (p4[2]-c)**2, r**2)

    sln_3 = sym.solve([eq1, eq2, eq3, eq4], (a, b, c, r))
    # print('\n3rd Combo sln')
    # print("The apple's center is located at (x, y, z): %.2f %.2f %.2f " % (sln_3[1][0], sln_3[1][1], sln_3[1][2]))
    # print("And it has a diameter of: %.3f cm" % (100 * 2 * sln_3[1][3]))


    # Combination 4
    p1 = app_p1
    p2 = app_p3
    p3 = app_p4
    p4 = app_p5

    # --- Step 3: Elaborate the system of equations, in this case the equation of a sphere
    eq1 = sym.Eq((p1[0]-a)**2 + (p1[1]-b)**2 + (p1[2]-c)**2, r**2)
    eq2 = sym.Eq((p2[0]-a)**2 + (p2[1]-b)**2 + (p2[2]-c)**2, r**2)
    eq3 = sym.Eq((p3[0]-a)**2 + (p3[1]-b)**2 + (p3[2]-c)**2, r**2)
    eq4 = sym.Eq((p4[0]-a)**2 + (p4[1]-b)**2 + (p4[2]-c)**2, r**2)

    sln_4 = sym.solve([eq1, eq2, eq3, eq4], (a, b, c, r))
    # print('\n4th Combo sln')
    # print("The apple's center is located at (x, y, z): %.2f %.2f %.2f " % (sln_4[1][0], sln_4[1][1], sln_4[1][2]))
    # print("And it has a diameter of: %.3f cm" % (100 * 2 * sln_4[1][3]))

    # Combination 5
    p1 = app_p2
    p2 = app_p3
    p3 = app_p4
    p4 = app_p5

    # --- Step 3: Elaborate the system of equations, in this case the equation of a sphere
    eq1 = sym.Eq((p1[0]-a)**2 + (p1[1]-b)**2 + (p1[2]-c)**2, r**2)
    eq2 = sym.Eq((p2[0]-a)**2 + (p2[1]-b)**2 + (p2[2]-c)**2, r**2)
    eq3 = sym.Eq((p3[0]-a)**2 + (p3[1]-b)**2 + (p3[2]-c)**2, r**2)
    eq4 = sym.Eq((p4[0]-a)**2 + (p4[1]-b)**2 + (p4[2]-c)**2, r**2)

    sln_5 = sym.solve([eq1, eq2, eq3, eq4], (a, b, c, r))
    # print('\n5th Combo sln')
    # print("The apple's center is located at (x, y, z): %.2f %.2f %.2f " % (sln_5[1][0], sln_5[1][1], sln_5[1][2]))
    # print("And it has a diameter of: %.3f cm" % (100 * 2 * sln_5[1][3]))


    a_avg = st.mean([float(sln_1[1][0]), float(sln_2[1][0]), float(sln_3[1][0]), float(sln_4[1][0]), float(sln_5[1][0])])
    b_avg = st.mean([float(sln_1[1][1]), float(sln_2[1][1]), float(sln_3[1][1]), float(sln_4[1][1]), float(sln_5[1][1])])
    c_avg = st.mean([float(sln_1[1][2]), float(sln_2[1][2]), float(sln_3[1][2]), float(sln_4[1][2]), float(sln_5[1][2])])
    r_avg = st.mean([float(sln_1[1][3]), float(sln_2[1][3]), float(sln_3[1][3]), float(sln_4[1][3]), float(sln_5[1][3])])


    a_stdev = st.stdev([float(sln_1[1][0]), float(sln_2[1][0]), float(sln_3[1][0]), float(sln_4[1][0]), float(sln_5[1][0])])
    b_stdev = st.stdev([float(sln_1[1][1]), float(sln_2[1][1]), float(sln_3[1][1]), float(sln_4[1][1]), float(sln_5[1][1])])
    c_stdev = st.stdev([float(sln_1[1][2]), float(sln_2[1][2]), float(sln_3[1][2]), float(sln_4[1][2]), float(sln_5[1][2])])
    r_stdev = st.stdev([float(sln_1[1][3]), float(sln_2[1][3]), float(sln_3[1][3]), float(sln_4[1][3]), float(sln_5[1][3])])

    print("\n------- Approach 1 -------")
    print("\n-- Average values:")
    print("Average of Apple's center (x, y, z): %.2f %.2f %.2f " % (a_avg, b_avg, c_avg))
    print("Average of Apple's diameter: %.2f cm" % (100 * 2 * r_avg))
    print("-- Standard deviations:")
    print("Std Dev of Apple's center (x, y, z): %.3f %.3f %.3f " % (a_stdev, b_stdev, c_stdev))
    print("Std Dev of Apple's diameter: %.3f cm\n" % (100 * 2 * r_stdev))


    # ---- Troubleshoot:
    # There might be points sampled which are close to the same plane, which would result in a sphere with a very big
    # radius, in that case, we disregard the combinations which

    if (sln_1[1][3] > (r_avg + r_stdev)) or (sln_1[1][3] < (r_avg - r_stdev)):
        print("Heads up!!!")
        print("combo 1 too weird, new values:")
        r_avg = st.mean([float(sln_2[1][3]), float(sln_3[1][3]), float(sln_4[1][3]), float(sln_5[1][3])])
        r_stdev = st.stdev([float(sln_2[1][3]), float(sln_3[1][3]), float(sln_4[1][3]), float(sln_5[1][3])])
        print("Average of Apple's diameter: %.2f cm" % (100 * 2 * r_avg))
        print("Std Dev of Apple's diameter: %.3f cm" % (100 * 2 * r_stdev))

    elif (sln_2[1][3] > (r_avg + r_stdev)) or (sln_1[1][3] < (r_avg - r_stdev)):
        print("Heads up!!!")
        print("combo 2 too weird, new values:")
        r_avg = st.mean([float(sln_1[1][3]), float(sln_3[1][3]), float(sln_4[1][3]), float(sln_5[1][3])])
        r_stdev = st.stdev([float(sln_1[1][3]), float(sln_3[1][3]), float(sln_4[1][3]), float(sln_5[1][3])])
        print("Average of Apple's diameter: %.2f cm" % (100 * 2 * r_avg))
        print("Std Dev of Apple's diameter: %.3f cm" % (100 * 2 * r_stdev))

    elif (sln_3[1][3] > (r_avg + r_stdev)) or (sln_3[1][3] < (r_avg - r_stdev)):
        print("Heads up!!!")
        print("combo 3 too weird, new values:")
        r_avg = st.mean([float(sln_1[1][3]), float(sln_2[1][3]), float(sln_4[1][3]), float(sln_5[1][3])])
        r_stdev = st.stdev([float(sln_1[1][3]), float(sln_2[1][3]), float(sln_4[1][3]), float(sln_5[1][3])])
        print("Average of Apple's diameter: %.2f cm" % (100 * 2 * r_avg))
        print("Std Dev of Apple's diameter: %.3f cm" % (100 * 2 * r_stdev))


    elif (sln_4[1][3] > (r_avg + r_stdev)) or (sln_4[1][3] < (r_avg - r_stdev)):
        print("Heads up!!!")
        print("combo 4 too weird, new values:")
        r_avg = st.mean([float(sln_1[1][3]), float(sln_2[1][3]), float(sln_3[1][3]), float(sln_5[1][3])])
        r_stdev = st.stdev([float(sln_1[1][3]), float(sln_2[1][3]), float(sln_3[1][3]), float(sln_5[1][3])])
        print("Average of Apple's diameter: %.2f cm" % (100 * 2 * r_avg))
        print("Std Dev of Apple's diameter: %.3f cm" % (100 * 2 * r_stdev))

    elif (sln_5[1][3] > (r_avg + r_stdev)) or (sln_5[1][3] < (r_avg - r_stdev)):
        print("\nHeads up!!!")
        print("combo 5 too weird, new values:")
        r_avg = st.mean([float(sln_1[1][3]), float(sln_2[1][3]), float(sln_3[1][3]), float(sln_4[1][3])])
        r_stdev = st.stdev([float(sln_1[1][3]), float(sln_2[1][3]), float(sln_3[1][3]), float(sln_4[1][3])])
        print("Average of Apple's diameter: %.2f cm" % (100 * 2 * r_avg))
        print("Std Dev of Apple's diameter: %.3f cm" % (100 * 2 * r_stdev))

def approach_2(app_p1, app_p2, app_p3, diameter, apple_number):
    """
    Obtain the center of a sphere, given 3 points and its diameter
    :param app_p1:
    :param app_p2:
    :param app_p3:
    :param diameter: Apple's diameter given in [m]
    :return:
    """




    # Combination 1
    p1 = app_p1
    p2 = app_p2
    p3 = app_p3
    radius = 1 * diameter / 2       # In [m]

    try:
        # --- Step 3: Elaborate the system of equations, in this case the equation of a sphere
        eq1 = sym.Eq((p1[0] - a) ** 2 + (p1[1] - b) ** 2 + (p1[2] - c) ** 2, radius ** 2)
        eq2 = sym.Eq((p2[0] - a) ** 2 + (p2[1] - b) ** 2 + (p2[2] - c) ** 2, radius ** 2)
        eq3 = sym.Eq((p3[0] - a) ** 2 + (p3[1] - b) ** 2 + (p3[2] - c) ** 2, radius ** 2)
        sln_1 = sym.solve([eq1, eq2, eq3], (a, b, c), warn=True)

        print("\n------- Approach 2 -------")
        print("The coordinates of the apple No. %i are" % (apple_number))
        print("Given Apple's diameter: %.2f cm" % (100 * 2 * radius))
        print("Average of Apple's center (x, y, z): %.2f %.2f %.2f " % (sln_1[1][0], sln_1[1][1], sln_1[1][2]))

        return sln_1[1][0], sln_1[1][1], sln_1[1][2]


    except TypeError or KeyError:
        print("exception")
        # Adjust a bit the radius, so it finds a solution
        radius = 1.01 * radius  # In [m]

        # --- Step 3: Elaborate the system of equations, in this case the equation of a sphere
        eq1 = sym.Eq((p1[0] - a) ** 2 + (p1[1] - b) ** 2 + (p1[2] - c) ** 2, radius ** 2)
        eq2 = sym.Eq((p2[0] - a) ** 2 + (p2[1] - b) ** 2 + (p2[2] - c) ** 2, radius ** 2)
        eq3 = sym.Eq((p3[0] - a) ** 2 + (p3[1] - b) ** 2 + (p3[2] - c) ** 2, radius ** 2)
        sln_1 = sym.solve([eq1, eq2, eq3], (a, b, c))

        print("\n------- Approach 2 -------")
        print(".... trying a bit bigger radius")
        print("The coordinates of the apple No. %i are" % (apple_number))
        print("Given Apple's diameter: %.2f cm" % (100 * 2 * radius))
        print("Average of Apple's center (x, y, z): %.2f %.2f %.2f " % (sln_1[1][0], sln_1[1][1], sln_1[1][2]))

        return sln_1[1][0], sln_1[1][1], sln_1[1][2]

def center_from_stem(stm_p1, stm_p2):

    a = (stm_p1[0] + stm_p2[0]) / 2
    b = (stm_p1[1] + stm_p2[1]) / 2
    c = (stm_p1[2] + stm_p2[2]) / 2

    return a, b, c

def check_dist_to_stem(a, b, c, stm_p1, stm_p2, diameter):
    """
    Checks if the stem is located within the apple
    :param a: Center (a,b,c)
    :param b:
    :param c:
    :param stm_p1: First point of Stem
    :param stm_p2: Second point of Stem
    :param diameter: Apple Diameter
    :return:
    """
    stem_vector = [stm_p2[0] - stm_p1[0], stm_p2[1] - stm_p1[1], stm_p2[2] - stm_p2[1]]
    print(stem_vector)


sym.init_printing()

# Lab's PC location
folder = '/home/avl/ur_ws/src/apple_proxy/coords/'
file = 'apple_scan'

apples = [0]*42

for i in range(1, len(apples) + 1):

    location = folder + file + str(i) + '.csv'

    # Lists to store the coordinates of the points
    app_p1 = []
    app_p2 = []
    app_p3 = []
    app_p4 = []
    app_p5 = []
    stm_p1 = []
    stm_p2 = []

    with open(location, newline='') as csv_file:
        reader = csv.reader(csv_file)
        next(reader, None)  # Skip the header.
        for row in (reader):
            app_p1 = [float(row[2]), float(row[3]), float(row[4])]
            app_p2 = [float(row[5]), float(row[6]), float(row[7])]
            app_p3 = [float(row[8]), float(row[9]), float(row[10])]
            app_p4 = [float(row[11]), float(row[12]), float(row[13])]
            app_p5 = [float(row[14]), float(row[15]), float(row[16])]
            stm_p1 = [float(row[17]), float(row[18]), float(row[19])]
            stm_p2 = [float(row[20]), float(row[21]), float(row[22])]
            app_diam = float(row[23])

    #print(app_p1, app_p2, app_p3, app_p4, app_p5, stm_p1, stm_p2)

    # --- Step 1: Define the variables of the Sphere
    a, b, c = sym.symbols('a, b, c')    # Coordinates of the center of the sphere / apple
    r = sym.Symbol('r')                 # Radius of the sphere / apple

    # Approach 1: Gets center and radius, given 4 points
    # approach_1(app_p1, app_p2, app_p3, app_p4, app_p5)

    # Approach 2: Gets center, given radius and 3 points
    # Because measuring the apple's diameter manually is easier than measuring 2 more points, and less prone to errors.
    a1, b1, c1 = approach_2(app_p1, app_p2, app_p3, app_diam/100, i)

    # Approach 3: Gets center, given the two points from the stem
    a2, b2, c2 = center_from_stem(stm_p1, stm_p2)

    # Finally average them to have a better result
    a = (a1 + a2) / 2
    b = (b1 + b2) / 2
    c = (c1 + c2) / 2

    print(a, b, c)
    apples[i-1] = [i, app_p1, app_p2, app_p3, app_diam/100, a, b, c, stm_p1, stm_p2]
    # Make list with all the apples


print('The characteristics of the real apples picked are: ', apples)
print('\nApple 2 scan: ', apples[1])

with open('apple_list.csv', 'w') as f:
    write = csv.writer(f)
    write.writerows(apples)
