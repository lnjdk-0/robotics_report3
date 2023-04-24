#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs

from tf.transformations import *
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Bool
from ur5e_control.msg import Plan
from geometry_msgs.msg import Twist
from robot_vision_lectures.msg import XYZarray
from robot_vision_lectures.msg import SphereParams
from std_msgs.msg import UInt8

# Global variables to store the detected ball and its state
ball = SphereParams()
detect_ball = False
start = Bool()
start = False
pause_task = False
#Edit 4/22/23
tool_pose = None

# Function to create a Twist message with given position and orientation values
def add_point(x, y, z, roll, pitch, yaw, grip):
    point = Twist()
    mode = UInt8()
    
    point.linear.x = x
    point.linear.y = y
    point.linear.z = z
    point.angular.x = roll
    point.angular.y = pitch
    point.angular.z = yaw
    mode.data = grip
    
    return point, mode

# Callback function for the SphereParams subscriber
def get_sphere(data):
    global ball
    global detect_ball
    global pause_task
    if not pause_task:
        ball.xc = data.xc
        ball.yc = data.yc
        ball.zc = data.zc
        ball.radius = data.radius
    detect_ball = True

# Callback function for the Bool subscriber to start the task
def start_task(bool):
    global start
    start = bool

# Callback function for the Bool subscriber to pause the task
def verify_ball(bool):
    #global pause_task ######Edited on 4/24/23 to prevent starting the task
    pause_task = bool
    
def get_toolpose(data):
	global tool_pose
	tool_pose = data

if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('simple_planner', anonymous=True)

    # Set up publisher, subscribers, and loop rate
    plan_pub = rospy.Publisher('/plan', Plan, queue_size=10)
    ball_sub = rospy.Subscriber('/sphere_params', SphereParams, get_sphere)
    bool_sub = rospy.Subscriber('/begin_task', Bool, start_task)
    track_sub = rospy.Subscriber('/pause_task', Bool, verify_ball)
    toolpose_sub = rospy.Subscriber('/ur5e/toolpose', Twist, get_toolpose)
    loop_rate = rospy.Rate(10)
    
    # Init tool pose
    t_pose = None

    # Initialize the tf2 buffer and listener
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)

    # Initialize quaternion and PointStamped variables for later use
    q_rot = Quaternion()
    cam = tf2_geometry_msgs.PointStamped()
    base = tf2_geometry_msgs.PointStamped()

    # Main loop
    while not rospy.is_shutdown():
    	if t_pose is None:
    		t_pose = tool_pose
    		
        if detect_ball and t_pose is not None:
            # Get the transformation between the base and camera_color_optical_frame
            try:
                trans = buffer.lookup_transform("base", "camera_color_optical_frame", rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print("Frames not available")
                continue

            # Extract translation and rotation from the transform
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z
            # Edit 4/24/23
            #q_rot = trans.transform.rotation
            #roll, pitch, yaw, = euler_from_quaternion([q_rot.x, q_rot.y, q_rot.z, q_rot.w])

            # Set up the PointStamped for the detected ball in the camera frame
            cam.header.frame_id = "camera_color_optical_frame"
            cam.header.stamp = rospy.get_rostime()
            cam.point.x = ball.xc
            cam.point.y = ball.yc
            cam.point.z = ball.zc

            # Transform the ball's point to the base frame
            base = buffer.transform(cam,'base', rospy.Duration(1.0))
            # Print the point coordinates in both the camera and base frames
            print('Test point in the CAM frame:  x= ', format(cam.point.x, '.3f'), '(m), y= ', format(cam.point.y, '.3f'), '(m), z= ', format(cam.point.z, '.3f'),'(m)')
            print('Transformed point in the BASE frame:  x= ', format(base.point.x, '.3f'), '(m), y= ', format(base.point.y, '.3f'), '(m), z= ', format(base.point.z, '.3f'),'(m)')
            print('-------------------------------------------------')
    
            # Define initial position and orientation values
            #rx, ry, rz = -0.13706, -0.50399, 0.43095
            #roll, pitch, yaw = 3.1, 0.0, 0.0
            rx, ry, rz = t_pose.linear.x, t_pose.linear.y, t_pose.linear.z
            roll, pitch, yaw = t_pose.angular.x, t_pose.angular.y, t_pose.angular.z
    
            # Initialize a Plan message and add points to the plan
            plan = Plan()
    
            # Initial robot pose
            point1, mode1 = add_point(rx, ry, rz, roll, pitch, yaw, 0)
            plan.points.append(point1)
            plan.modes.append(mode1)
    
            # Above the ball
            point2, mode2 = add_point(base.point.x, base.point.y, base.point.z + ball.radius + 0.1, roll, pitch, yaw, 0)
            plan.points.append(point2)
    
            # Touching the ball
            point3, mode3 = add_point(base.point.x, base.point.y, base.point.z + ball.radius, roll, pitch, yaw, 2)
            plan.points.append(point3)
    
            # Go up
            plan.points.append(point2)
    
            # Move to another position
            point4, mode4 = add_point(base.point.x + .2, base.point.y + .05, base.point.z + ball.radius + 0.1, roll, pitch, yaw)
            plan.points.append(point4)
    
            # Lower the arm
            point5, mode5 = add_point(base.point.x + .2, base.point.y + .05, base.point.z + ball.radius, roll, pitch, yaw, 1)
            plan.points.append(point5)
    
            # Go up
            plan.points.append(point4)

            # Publish the plan if the task has started
            if start:
                plan_pub.publish(plan)
            else:
                print("Awaiting for start")
    
        # Sleep for the loop rate
        loop_rate.sleep()
