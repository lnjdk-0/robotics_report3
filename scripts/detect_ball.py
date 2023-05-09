#!/usr/bin/env python3

"""
A ROS node that subscribes to an image topic, processes the image by detecting a ball in the image,
and publishes a mono-channel image with the detected ball.

@author: Leon Neverov
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Global variables
img_received = False
rgb_img = np.zeros((720, 1280, 3), dtype="uint8")

# Callback function for the image subscriber
def get_image(ros_img):
    global rgb_img
    global img_received

    # Convert ROS image message to OpenCV image (RGB)
    rgb_img = CvBridge().imgmsg_to_cv2(ros_img, "rgb8")

    # Set flag to indicate that an image has been received
    img_received = True

# Function to apply a mask to the image
def get_mask(image):
    filter = np.zeros((720, 1280), dtype="uint8")
    cv2.rectangle(filter, (300, 160), (720, 1000), (255, 255, 255), -1)
    mask = cv2.bitwise_and(image, filter)
    return mask

# Function to filter the image and detect the ball
def get_filter(image):
    # Convert the image to HSV color space
    hsv_img = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

    # Define color range for the tennis ball (yellow) in HSV
    lower_yellow = np.array([20, 0, 0])
    upper_yellow = np.array([60, 255, 255])

    # Apply the color range to create a mask
    yellow_mask = cv2.inRange(hsv_img, lower_yellow, upper_yellow)

    return yellow_mask

# Main function
if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('detect_ball', anonymous=True)
    
    # Subscribe to the image topic
    img_sub = rospy.Subscriber("/camera/color/image_raw", Image, get_image)
    
    # Define a publisher to publish the processed image
    img_pub = rospy.Publisher('/ball_2D', Image, queue_size=1)
    
    # Set the control loop frequency
    rate = rospy.Rate(10)
    
    # Main control loop
    while not rospy.is_shutdown():
        # Check if an image has been received
        if img_received:
            # Filter the image to detect the ball
            ball = get_filter(rgb_img)
            ball = get_mask(ball)

            # Convert the OpenCV image to a ROS image message
            img_msg = CvBridge().cv2_to_imgmsg(ball, encoding="mono8")
            
            # Publish the processed image
            img_pub.publish(img_msg)
        
        # Sleep for the duration required to maintain the desired loop frequency
        rate.sleep()
