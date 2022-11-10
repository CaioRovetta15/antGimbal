# /usr/bin/python3

import rospy 
from aruco_cube import ArucoCube
from camera import Camera

if __name__ == '__main__':

    # init node
    rospy.init_node('antenna_controller')
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

        # detect cube and estimate pose of the cube 
        # gettting the transformation matrix
        trans = cube.detect(frame)

        # TODO: inverse kinematics
        # q = inverse_kinematics(trans) 

        # TODO: send joint angles to esp32
        # send_joint_angles(q)

        # sleep
        rate.sleep()

    