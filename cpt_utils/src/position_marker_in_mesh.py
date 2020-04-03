#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Transform, Vector3, Quaternion, Point
from visualization_msgs.msg import InteractiveMarkerControl
from tf.broadcaster import TransformBroadcaster
from std_srvs.srv import Empty, EmptyRequest

from cpt_utils.marker import RosMarker

def frame_callback(marker_msg, tf_broadcaster, map_frame):
    time = rospy.Time.now()
    pose = marker_msg.pose
    tf_broadcaster.sendTransform(
        (pose.position.x, pose.position.y, pose.position.z),
        (pose.orientation.x, pose.orientation.y, pose.orientation.z,
         pose.orientation.w),
        time, rospy.get_param('~marker_frame_name'), map_frame)

if __name__ == "__main__":
    rospy.init_node("Marker Pose")
    transform_pub = rospy.Publisher('/T_map_marker', Transform, queue_size=10)
    tf_broadcaster = TransformBroadcaster()
    marker = RosMarker(rospy.get_param('~marker_name'),
                       'Move this marker',
                       'marker_pose',
                       InteractiveMarkerControl.MOVE_ROTATE_3D,
                       show_controls=True,
                       position=[0, 0, 0],
                       orientation=[0, 0, 0, 1])

    # Create a timer to regularly query for new IDs
    rospy.Timer(rospy.Duration(0.01), lambda msg: frame_callback(
        marker.marker, tf_broadcaster, rospy.get_param('~marker_parent_frame')))

    rospy.spin()
