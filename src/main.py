#! /usr/bin/python3

import rospy 
import cv2
import time
import numpy as np
import threading
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from statistics import mean 

# local modules
from aruco_cube import ArucoCube
from camera import Camera
import socket_server 
import tf_publisher
import kinematics

bridge = CvBridge()

# Publish image debug of the cube
def publishImageDebug(frame):
    image_cube_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))

if __name__ == '__main__':

    # init node
    print("Starting node")
    rospy.init_node('antenna_controller', disable_signals=True)
    rate = rospy.Rate(100)
    tf_publisher.init_tf_buffer()

    # Create image debug publisher
    image_cube_pub = rospy.Publisher('aruco_cube/image_debug', Image, queue_size=1)

    # create camera and cube
    cam = Camera(2, 60, width=1280, height=720)
    cube = ArucoCube()

    # start camera
    cam.startCam()

    # start robot kinematics
    robot = kinematics.DHRobot()

    # start socket thread to send
    thrd = threading.Thread(target=socket_server.connectESP32, args=(8890,))
    thrd.start() 
        
    yaw_vector = []        
    pitch_vector = []
    # main loop
    while not rospy.is_shutdown():
        # get frame
        frame = cam.getFrame()
        if frame is None:
            continue

        # detect cube and estimate pose of the cube 
        # getting the transformation matrix
        T_cam_cube = cube.detect(frame)

        if T_cam_cube is not None:
            # draw cube on frame 
            frame = cube.drawFaces(frame)
            publishImageDebug(frame)

            # send transformations to tf
            tf_publisher.sendAllTransforms([T_cam_cube], ['camera_optical'], ['cube'])

            T_robot_target = tf_publisher.getTransform('base_link', 'target')
            # print(T_robot_target)
            # get the angles of the robot
            q = kinematics.inverse_kinematics(T_robot_target,robot)
            robot.q = q
            
            yaw_vector.append(np.rad2deg(q[0]) + 94)
            pitch_vector.append(np.rad2deg(q[1]) + 87)

            if len(yaw_vector) > 10 : yaw_vector.pop(0)
            if len(pitch_vector) > 10 : pitch_vector.pop(0)

            # send joint angles to esp32
            angle1 = int(round(mean(yaw_vector), 0))
            angle2 = int(round(mean(pitch_vector), 0))
            socket_server.send_joint_angles(angle1, angle2)

            
        else:
            publishImageDebug(frame)
            
        # cv2.imshow("frame", frame)

        # sleep
        rate.sleep()
        
    