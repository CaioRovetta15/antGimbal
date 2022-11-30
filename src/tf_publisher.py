# /usr/bin/python3

# This file is a collection of function to send the transformations and the image via ROS, using TF library making it possible to use Rviz to analyse the results

import rospy
import tf2_ros
import tf.transformations
import numpy as np

# import messages
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3

# Create a TF broadcaster
br = tf2_ros.TransformBroadcaster()

 # Create a TF listener
tfBuffer = tf2_ros.Buffer()

# Latch list to not publish same data on tf
T_list_old = []

# This function sends the transformation matrix using TF library
def sendAllTransforms(T_list, frame_ids, parent_frame_ids):
    
    if len(T_list) != len(frame_ids) or len(T_list) != len(parent_frame_ids):
        # raise exception
        raise Exception("The length of the T_list, frame_ids and parent_frame_ids must be the same")

    # Create a list of TF messages
    tf_list = []
    global T_list_old

    # Create a TF message for each transformation matrix
    for i in range(len(T_list)):
        if (i < len(T_list_old)):
            old = np.array(T_list_old[i], dtype=np.float16)
            new = np.array(T_list[i], dtype=np.float16)
            if (np.allclose(old, new)):
                print("Same data. Ignoring.")
                break

        tf_msg = TransformStamped()
        tf_msg.header.stamp = rospy.Time.now()
        tf_msg.header.frame_id = frame_ids[i]
        tf_msg.child_frame_id = parent_frame_ids[i]
        tf_msg.transform.translation.x = T_list[i][0][3]
        tf_msg.transform.translation.y = T_list[i][1][3]
        tf_msg.transform.translation.z = T_list[i][2][3]
        
        q = tf.transformations.quaternion_from_matrix(T_list[i])
        tf_msg.transform.rotation.x = q[0]
        tf_msg.transform.rotation.y = q[1]
        tf_msg.transform.rotation.z = q[2]
        tf_msg.transform.rotation.w = q[3]
        tf_list.append(tf)

        # send the TF message
        br.sendTransform(tf_msg)
    
    
    T_list_old = T_list
    return

def getTransform(frame_id, child_frame_id):
    
    listener = tf2_ros.TransformListener(tfBuffer)
    
    # Get the transformation matrix
    try:
        trans = tfBuffer.lookup_transform(frame_id, child_frame_id, rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        # print(e)
        return None
        
    # Convert the transformation matrix to a numpy array
    T = msg_to_se3(trans)
    return T

def pose_to_pq(msg):
    """Convert a C{geometry_msgs/Pose} into position/quaternion np arrays

    @param msg: ROS message to be converted
    @return:
      - p: position as a np.array
      - q: quaternion as a numpy array (order = [x,y,z,w])
    """
    p = np.array([msg.position.x, msg.position.y, msg.position.z])
    q = np.array([msg.orientation.x, msg.orientation.y,
                  msg.orientation.z, msg.orientation.w])
    return p, q


def pose_stamped_to_pq(msg):
    """Convert a C{geometry_msgs/PoseStamped} into position/quaternion np arrays

    @param msg: ROS message to be converted
    @return:
      - p: position as a np.array
      - q: quaternion as a numpy array (order = [x,y,z,w])
    """
    return pose_to_pq(msg.pose)


def transform_to_pq(msg):
    """Convert a C{geometry_msgs/Transform} into position/quaternion np arrays

    @param msg: ROS message to be converted
    @return:
      - p: position as a np.array
      - q: quaternion as a numpy array (order = [x,y,z,w])
    """
    p = np.array([msg.translation.x, msg.translation.y, msg.translation.z])
    q = np.array([msg.rotation.x, msg.rotation.y,
                  msg.rotation.z, msg.rotation.w])
    return p, q


def transform_stamped_to_pq(msg):
    """Convert a C{geometry_msgs/TransformStamped} into position/quaternion np arrays

    @param msg: ROS message to be converted
    @return:
      - p: position as a np.array
      - q: quaternion as a numpy array (order = [x,y,z,w])
    """
    return transform_to_pq(msg.transform)


def msg_to_se3(msg):
    """Conversion from geometric ROS messages into SE(3)

    @param msg: Message to transform. Acceptable types - C{geometry_msgs/Pose}, C{geometry_msgs/PoseStamped},
    C{geometry_msgs/Transform}, or C{geometry_msgs/TransformStamped}
    @return: a 4x4 SE(3) matrix as a numpy array
    @note: Throws TypeError if we receive an incorrect type.
    """
    if isinstance(msg, Pose):
        p, q = pose_to_pq(msg)
    elif isinstance(msg, PoseStamped):
        p, q = pose_stamped_to_pq(msg)
    elif isinstance(msg, Transform):
        p, q = transform_to_pq(msg)
    elif isinstance(msg, TransformStamped):
        p, q = transform_stamped_to_pq(msg)
    else:
        raise TypeError("Invalid type for conversion to SE(3)")
    norm = np.linalg.norm(q)
    if np.abs(norm - 1.0) > 1e-3:
        raise ValueError(
            "Received un-normalized quaternion (q = {0:s} ||q|| = {1:3.6f})".format(
                str(q), np.linalg.norm(q)))
    elif np.abs(norm - 1.0) > 1e-6:
        q = q / norm
    g = tf.transformations.quaternion_matrix(q)
    g[0:3, -1] = p
    return g