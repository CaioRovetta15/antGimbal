# /usr/bin/python3

# This class set the camera parameters, start stream with a rostimer and publish Image on ROS

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import traceback

bridge = CvBridge()

class Camera:
    def __init__(self, device_id:int=0, fps:float=30, width:int=640, height:int=480, autofocus:bool=True) -> None:
        
        # device id
        self.device_id = device_id
        
        # camera fps
        self.fps = fps
        
        # resolution
        self.width = height
        self.height = width

        # autofocus
        self.autofocus = autofocus

        # camera obj
        self.cam = None
        self.frame = None

        # ros objs
        self.cam_pub = None

    def startCam(self):

        # Initialize the video stream and allow the camera sensor to warm up
        print("[INFO] starting video stream...")
        self.cam = cv2.VideoCapture(self.device_id)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cam.set(cv2.CAP_PROP_FPS, self.fps)
        self.cam.set(cv2.CAP_PROP_AUTOFOCUS, self.autofocus)
        self.cam.set(cv2.CAP_PROP_FOCUS, self.autofocus)

        # start stream and publish it to ROS for debug purposes
        self.cam_pub = rospy.Publisher('camera/image_raw', Image, queue_size=1)
        rospy.Timer(rospy.Duration(1.0/self.fps), self.__readAndPublish, reset=True)

    def __readAndPublish(self):
        try:
            ret, frame = self.cam.read()
            
            # check if frame is ok
            if not ret:
                rospy.logerr("Error reading frame from camera")
                return False

            # update frame atribute
            self.frame = frame

            # publish on ros
            bridge.cv2_to_imgmsg(self.frame, encoding="bgr8")
            self.cam_pub.publish()
        except Exception as e:
            # When everything done, release the capture
            self.cam.release()
            cv2.destroyAllWindows()

            # print error
            traceback.print_exc()

    def getFrame(self, cam):
        return self.frame