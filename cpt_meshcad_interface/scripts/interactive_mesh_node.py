#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Transform, Vector3, Quaternion
from visualization_msgs.msg import InteractiveMarkerControl
from tf.broadcaster import TransformBroadcaster
from std_srvs.srv import Empty, EmptyRequest

from cpt_meshcad_interface.marker import RosMarker


def frameCallback(marker_msg, tf_broadcaster, map_frame):
    time = rospy.Time.now()
    pose = marker_msg.pose
    tf_broadcaster.sendTransform(
        (pose.position.x, pose.position.y, pose.position.z),
        (pose.orientation.x, pose.orientation.y, pose.orientation.z,
         pose.orientation.w),
        time, 'marker_position', map_frame)


def loadCADCb(transform_pub, marker_msg):
    rospy.loginfo("Loading CAD.")

    rospy.wait_for_service('/mapper/load_published_map')
    try:
        load_cad_proxy = rospy.ServiceProxy('/mapper/load_published_map',
                                            Empty)
        success = load_cad_proxy(EmptyRequest())
        if not success:
            print("Loading CAD did not work.")
        else:
            print("Loading CAD successful.")
            pose = marker_msg.pose
            transform = Transform()
            transform.translation = Vector3(
                pose.position.x, pose.position.y, pose.position.z)
            transform.rotation = Quaternion(pose.orientation.x, pose.orientation.y,
                                            pose.orientation.z, pose.orientation.w)
            transform_pub.publish(transform)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


if __name__ == "__main__":
    rospy.init_node("Interactive_mesh_postitioning")
    transform_pub = rospy.Publisher('/T_map_marker', Transform, queue_size=10)
    tf_broadcaster = TransformBroadcaster()

    marker = RosMarker('BuildingModel',
                       'Position this marker to initialize\n'
                       'the position of the building model.',
                       'interactive_mesh_marker_controls',
                       InteractiveMarkerControl.MOVE_ROTATE_3D, show_6dof=True)
    marker.add_menu_item('Load CAD', lambda msg: loadCADCb(transform_pub, marker.marker))

    # Create a timer to update the published transforms
    rospy.Timer(rospy.Duration(0.01), lambda msg: frameCallback(
        marker.marker, tf_broadcaster, 'map'))

    rospy.spin()
