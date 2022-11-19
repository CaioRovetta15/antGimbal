#! /usr/bin/python3

import rospy 
import cv2

# local modules
from aruco_cube import ArucoCube
from camera import Camera
import tf_publisher
import kinematics

if __name__ == '__main__':

    # init node
    print("Starting node")
    rospy.init_node('antenna_controller', disable_signals=True)
    rate = rospy.Rate(100)

    # create camera and cube
    cam = Camera()
    cube = ArucoCube()

    # start camera
    cam.startCam()
    robot = kinematics.DHRobot()
    # main loop
    while not rospy.is_shutdown():
        # get frame
        frame = cam.getFrame()
        if frame is None:
            continue

        # detect cube and estimate pose of the cube 
        # getting the transformation matrix
        trans = cube.detect(frame)

        if trans:
            # draw cube on frame 
            frame = cube.drawFaces(frame)

            q = kinematics.inverse_kinematics(robot,trans) 

            # TODO: send joint angles to esp32
            # send_joint_angles(q)

            # send transformations to tf
            tf_publisher.sendAllTransforms([trans], ['camera_optical'], ['cube'])

        cv2.imshow("frame", frame)

        # sleep
        rate.sleep()
        cv2.waitKey(1)
    