# /usr/bin/python3

import rospy 
from aruco_cube import ArucoCube
from camera import Camera
import cv2

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

    # main loop
    while not rospy.is_shutdown():
        # get frame
        frame = cam.getFrame()

        if frame is None:
            continue
        
        # detect cube and estimate pose of the cube 
        # gettting the transformation matrix
        trans = cube.detect(frame)

        frame = cube.drawFaces(frame)

        cv2.imshow("frame", frame)

        # sleep
        rate.sleep()
        cv2.waitKey(1)

    