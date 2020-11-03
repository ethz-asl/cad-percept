#!/usr/bin/env python
from os import path, mkdir

import rospy
import tf
from tf.listener import TransformListener
from geometry_msgs.msg import PoseStamped



if __name__ == "__main__":
    rospy.init_node("msg_hacker")

    listener = TransformListener()
    msg_pub = rospy.Publisher('/state_estimator/smb_pose', PoseStamped, queue_size=10)

    def publish(*args):
        while True:
            try:
                (trans, rot) = listener.lookupTransform('marker', 'base', rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        pose_msg = PoseStamped()
        pose_msg.pose.position.x = trans[0]
        pose_msg.pose.position.y = trans[1]
        pose_msg.pose.position.z = trans[2]
        pose_msg.pose.orientation.x = rot[0]
        pose_msg.pose.orientation.y = rot[1]
        pose_msg.pose.orientation.z = rot[2]
        pose_msg.pose.orientation.w = rot[3]
        msg_pub.publish(pose_msg)
    # Create a timer to update the published transforms
    rospy.Timer(rospy.Duration(0.01), publish)

    # republish mesh once when everything is set up
    rospy.sleep(1)

    rospy.spin()
