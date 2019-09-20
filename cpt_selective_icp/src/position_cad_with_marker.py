#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Transform, Vector3, Quaternion
from visualization_msgs.msg import InteractiveMarkerControl
from tf.broadcaster import TransformBroadcaster
from std_srvs.srv import Empty, EmptyRequest

from cgal_msgs.srv import PublishMesh
from cpt_utils.marker import RosMarker


def frame_callback(marker_msg, tf_broadcaster, map_frame):
    time = rospy.Time.now()
    pose = marker_msg.pose
    tf_broadcaster.sendTransform(
        (pose.position.x, pose.position.y, pose.position.z),
        (pose.orientation.x, pose.orientation.y, pose.orientation.z,
         pose.orientation.w),
        time, 'marker_position', map_frame)


if __name__ == "__main__":
    rospy.init_node("mesh_postitioning")
    transform_pub = rospy.Publisher('/T_map_marker', Transform, queue_size=10)
    tf_broadcaster = TransformBroadcaster()

    marker = RosMarker('BuildingModel',
                       'Position this marker to initialize\n'
                       'the position of the building model.',
                       'mesh_position_marker',
                       InteractiveMarkerControl.MOVE_ROTATE_3D, show_controls=True,
                       parent_frame='map')

    publish_mesh = rospy.ServiceProxy('/mesh_publisher/publish', PublishMesh)
    marker.add_menu_item('republish mesh', lambda msg: publish_mesh())

    load_cad = rospy.ServiceProxy('/mapper/load_published_map', Empty)
    marker.add_menu_item('load CAD', lambda msg: load_cad())

    # Create a timer to update the published transforms
    rospy.Timer(rospy.Duration(0.01), lambda msg: frame_callback(
        marker.marker, tf_broadcaster, 'map'))

    # republish mesh once when everything is set up
    rospy.sleep(1)
    publish_mesh()

    rospy.spin()
