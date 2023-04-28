#!/usr/bin/env python3
"""
A ROS node that fits a sphere to a set of 3D points received from a subscriber.
The fitted sphere parameters (center coordinates and radius) are published to a topic.
@author Leon Neverov
@credits https://github.com/hsaeidi-uncw/robot_vision_lectures
"""

import numpy as np
import rospy
from robot_vision_lectures.msg import XYZarray, SphereParams

msg_received = False

def get_msg(data):
    """
       Callback function for the 'xyz_cropped_ball' subscriber. Extracts and stores the 3D points.
       Args:
           data (XYZarray): The received message containing the 3D points.
    """


    global msg_received
    global point_arr
    msg_received = True
    point_arr = [(point.x, point.y, point.z) for point in data.points]


def get_radius(POINTS):
    """
    Calculates the radius of the fitted sphere.
    Args:
        POINTS (numpy.ndarray): The fitted sphere parameters.
    Returns:
        radius (float): The radius of the fitted sphere.
    """


    xc, yc, zc, _ = POINTS[0]
    radius = np.sqrt(POINTS[0][3] + xc ** 2 + yc ** 2 + zc ** 2)
    return radius


def sphere_fit(points):
    """
    Fits a sphere to the given 3D points using linear least squares.
    Args:
        points (list): A list of 3D points as tuples (x, y, z).
    Returns:
        POINTS (numpy.ndarray): The fitted sphere parameters.
    """


    A = []
    B = []

    for point in points:
        x, y, z = point
        B.append([x ** 2 + y ** 2 + z ** 2])
        A.append([2 * x, 2 * y, 2 * z, 1])

    A = np.array(A)
    B = np.array(B)

    B = B.reshape(len(B), 1)
    A = A.reshape(len(B), 4)

    POINTS = np.linalg.lstsq(A, B, rcond=None)
    return POINTS
    
def low_pass_filter(current_value, previous_value, alpha):
    """
    Applies a low-pass filter to the input values.
    Args:
        current_value (float): The current value to be filtered.
        previous_value (float): The previous value before filtering.
        alpha (float): The filter gain, between 0 and 1.
    Returns:
        filtered_value (float): The filtered output value.
    """
    return (1 - alpha) * previous_value + alpha * current_value

if __name__ == '__main__':
    # define the node and subcribers and publishers
    rospy.init_node('sphere_fit', anonymous=True)
    # define a subscriber to XYZ array
    sub = rospy.Subscriber('xyz_cropped_ball', XYZarray, get_msg)
    # define a publisher to publish Sphere params
    pub = rospy.Publisher("/sphere_params", SphereParams, queue_size=1)
    # set the loop frequency
    rate = rospy.Rate(10)

    # Initialize previous values for low-pass filter
    prev_xc, prev_yc, prev_zc, prev_radius = 0, 0, 0, 0

    # Set filter gain (alpha) between 0 and 1
    alpha = 0.02
    
    # Add a flag to check for the first measurement
    first_measurement = True

    while not rospy.is_shutdown():
        if msg_received:
            POINTS = sphere_fit(point_arr)
            radius = get_radius(POINTS)
            xc, yc, zc, _ = POINTS[0]

            # Apply low-pass filter to the sphere parameters
            filtered_xc = low_pass_filter(xc, prev_xc, alpha)
            filtered_yc = low_pass_filter(yc, prev_yc, alpha)
            filtered_zc = low_pass_filter(zc, prev_zc, alpha)
            filtered_radius = low_pass_filter(radius, prev_radius, alpha)

            # Update previous values for the next iteration
            prev_xc, prev_yc, prev_zc, prev_radius = filtered_xc, filtered_yc, filtered_zc, filtered_radius

            sphere_params = SphereParams(float(filtered_xc), float(filtered_yc), float(filtered_zc), filtered_radius)
            pub.publish(sphere_params)

        rate.sleep()
