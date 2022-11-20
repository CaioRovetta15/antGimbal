#! /usr/bin/python3

import rospy 
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# local modules
from aruco_cube import ArucoCube
from camera import Camera
import tf_publisher

bridge = CvBridge()

# Publish image debug of the cube
def publishImageDebug(frame):
    image_cube_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))

if __name__ == '__main__':

    # init node
    print("Starting node")
    rospy.init_node('antenna_controller', disable_signals=True)
    rate = rospy.Rate(100)

    # Create image debug publisher
    image_cube_pub = rospy.Publisher('aruco_cube/image_debug', Image, queue_size=1)

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

        if trans is not None:
            # draw cube on frame 
            frame = cube.drawFaces(frame)
            publishImageDebug(frame)

            # TODO: inverse kinematics
            # q = inverse_kinematics(trans) 

            # TODO: send joint angles to esp32
            # send_joint_angles(q)

            # send transformations to tf
            tf_publisher.sendAllTransforms([trans], ['camera_optical'], ['cube'])
        else:
            publishImageDebug(frame)
            
        # cv2.imshow("frame", frame)

        # sleep
        rate.sleep()
        cv2.waitKey(1)
    