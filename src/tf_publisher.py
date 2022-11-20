# /usr/bin/python3

# This file is a collection of function to send the transformations and the image via ROS, using TF library making it possible to use Rviz to analyse the results

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf.transformations

# This function sends the transformation matrix using TF library
def sendAllTransforms(T_list, frame_ids, parent_frame_ids):
    
    if len(T_list) != len(frame_ids) or len(T_list) != len(parent_frame_ids):
        # raise exception
        raise Exception("The length of the T_list, frame_ids and parent_frame_ids must be the same")

    # Create a TF broadcaster
    br = tf2_ros.TransformBroadcaster()

    # Create a list of TF messages
    tf_list = []

    # Create a TF message for each transformation matrix
    for i in range(len(T_list)):
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
        br.sendTransform(tf_msg)
    
    # Send all TF messages
    # br.sendTransform(tf_list)
