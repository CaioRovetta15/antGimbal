#/usr/bin/python3

# This file takes a camera stream, estimate the pose of a cube of aruco markers and return the transformation matrix

import cv2
import cv2.aruco as aruco
import numpy as np
from geometry_msgs.msg import TransformStamped

class ArucoCube:

    def __init__(self) -> None:
        # Camera calibration parameters
        self.camera_matrix = np.array([[725.7657025001567, 0.0, 302.64698191622074], [0.0, 733.3294790963405, 277.5564048217287], [0.0, 0.0, 1.0]])
        self.dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

        # Load the dictionary that was used to generate the markers.
        self.DICTIONARY = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

        # Constant parameters used in Aruco methods
        self.ARUCO_PARAMETERS = aruco.DetectorParameters_create()

        # Cube dimensions
        self.marker_length = 0.05
        self.marker_ids = [0, 1, 2, 3, 4]

        # Create vectors we'll be using for rotations and translations for postures
        rvecs, tvecs = None, None

    def estimateCubePose(self, img: np.array):
        
        # estimate pose of a single aruco
        corners, ids, rejectedImgPoints = aruco.detectMarkers(self.cam, self.DICTIONARY, parameters=self.ARUCO_PARAMETERS)

        # If at least one marker detected
        if ids is not None and len(ids) > 0:
            # Estimate the posture of the single marker
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.camera_matrix, self.dist_coeffs)

            # Unpack the output, get only the first
            # rvec, tvec = rvec[0, 0, :], tvec[0, 0, :]

            for index, rvec, tvec in enumerate(zip(rvecs, tvecs)):
                img = aruco.drawAxis(img, self.camera_matrix, self.dist_coeffs, rvec, tvec, 1)


            return (rvecs, tvecs)

    def detect(self, frame: np.array):

        # detect cube and estimate pose of each face
        trans = self.estimateCubePose(frame)

        # return transformation matrix 
        return trans