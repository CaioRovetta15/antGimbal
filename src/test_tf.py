# /usr/bin/python3

# testing tf_publisher and specialities of tf library

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

rospy.init_node("test_tf", disable_signals=True)
br = tf2_ros.TransformBroadcaster()

while not rospy.is_shutdown():
    # tf_msg = TransformStamped()
    # tf_msg.header.stamp = rospy.Time.now()
    # tf_msg.header.frame_id = "world"
    # tf_msg.child_frame_id = "cube_face_0"
    # tf_msg.transform.translation.x = 0.0
    # tf_msg.transform.translation.y = 1.0
    # tf_msg.transform.translation.z = 1.0

    # tf_msg.transform.rotation.x = 0.0
    # tf_msg.transform.rotation.y = 0.0
    # tf_msg.transform.rotation.z = 0.0
    # tf_msg.transform.rotation.w = 1.0

    # br.sendTransform(tf_msg)

    tf_msg = TransformStamped()
    tf_msg.header.stamp = rospy.Time.now()
    tf_msg.header.frame_id = "cube_face_0"
    tf_msg.child_frame_id = "cube"
    tf_msg.transform.translation.x = 0.0
    tf_msg.transform.translation.y = 1.0
    tf_msg.transform.translation.z = 1.0

    tf_msg.transform.rotation.x = 0.0
    tf_msg.transform.rotation.y = 0.0
    tf_msg.transform.rotation.z = 0.0
    tf_msg.transform.rotation.w = 1.0

    br.sendTransform(tf_msg)

    rospy.sleep(rospy.Duration(0.5))