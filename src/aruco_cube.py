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
        self.marker_length = 0.0525
        self.marker_ids = [0, 1, 2, 3, 4]

        # Create vectors we'll be using for rotations and translations for postures
        rvecs, tvecs = None, None

        # Cube state
        self.corners = None
        self.ids = None

        self.createBoard()

    def createBoard(self):
        l = self.marker_length
        
        # Object points of each aruco on the cube coordinate frame
        # The order of the faces is important, it must be the same as the order of the ids
        # For each face, the vertices of each corner of the aruco must be specified with 3D coordinates in the right order
        # The order of the corners is defined clockwise. So if you know wich is the corner 0, you can deduce the other corners
        # So, the obj_points and ids matrix will be:
        # 
        # obj_points:
        # [[[corner0][corner1][corner2][corner3]] # Aruco ID 0
        #  [[corner0][corner1][corner2][corner3]] # Aruco ID 1
        #  ...
        #  [[corner0][corner1][corner2][corner3]]] # Aruco ID N
        # 
        # each [cornerX] = [[x], [y], [z]]
        # 
        # ids: (same order of obj_points)
        # [0, 1, ..., N]
        obj_points = np.array([[[l/2, -l/2, l/2],[l/2, l/2, l/2],[l/2, l/2, -l/2],[l/2, -l/2, -l/2]],
                                [[l/2, l/2, l/2],[-l/2, l/2, l/2], [-l/2, l/2, -l/2], [l/2, l/2, -l/2]],
                                [[l/2, -l/2, -l/2],[-l/2, -l/2, -l/2],[-l/2, -l/2, l/2],[l/2, -l/2, l/2]],
                                [[-l/2, l/2, l/2],[-l/2, -l/2, l/2],[-l/2, -l/2, -l/2],[-l/2, l/2, -l/2]],
                                [[l/2, -l/2, l/2],[-l/2, -l/2, l/2],[-l/2, l/2, l/2],[l/2, l/2, l/2]]])
        
        
        ids = np.array([0, 1, 2, 3, 4])

        # create board
        self.cube_board = cv2.aruco.Board_create(obj_points.astype(np.float32), self.DICTIONARY, ids)

        return

    def estimateSinglePose(self, frame: np.array):
        
        # estimate pose of the arucos present on the image
        self.corners, self.ids, rejectedImgPoints = aruco.detectMarkers(frame, self.DICTIONARY, parameters=self.ARUCO_PARAMETERS)

        # If at least one marker detected
        if self.ids is not None and len(self.ids) > 0:
            # Estimate the posture of the single marker
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(self.corners, self.marker_length, self.camera_matrix, self.dist_coeffs)

            # Unpack the output, get only the first
            # rvec, tvec = rvec[0, 0, :], tvec[0, 0, :]

            for index, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
                frame = aruco.drawAxis(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.03)


            return (rvecs, tvecs)

    def estimateCubePose(self, frame):
        # estimate pose of the arucos present on the image
        self.corners, self.ids, rejectedImgPoints = aruco.detectMarkers(frame, self.DICTIONARY, parameters=self.ARUCO_PARAMETERS)

        if self.ids is None:
            return None

        # estimate pose of the cube
        rvec_temp = np.ascontiguousarray([0,0,0], dtype=np.float64)
        tvec_temp = np.ascontiguousarray([0,0,0], dtype=np.float64)
        retval, rvec, tvec = aruco.estimatePoseBoard(self.corners, self.ids, self.cube_board, self.camera_matrix, self.dist_coeffs, rvec_temp, tvec_temp)

        # if the cube is not detected
        if not retval:
            return None
        
        frame = cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.03)

        # # get the transformation
        T = cv2.Rodrigues(rvec)[0]
        T = np.append(T, np.expand_dims(tvec,1), axis=1)
        T = np.append(T, np.array([[0, 0, 0, 1]]), axis=0)

        # print(T)
        return T

    def detect(self, frame: np.array):

        # detect cube and estimate pose of each face
        trans = self.estimateCubePose(frame)

        # return transformation matrix 
        return trans
    
    def drawFaces(self, frame: np.array):
        
        if self.corners is None or self.ids is None:
            return frame

        # draw arucos on the frame
        frame = aruco.drawDetectedMarkers(frame, self.corners, self.ids)

        # draw corners ids on the frame
        for i in range(len(self.ids)):
            for j in range(4):
                cv2.putText(frame, str(j), (int(self.corners[i][0][j][0]), int(self.corners[i][0][j][1])), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

        return frame