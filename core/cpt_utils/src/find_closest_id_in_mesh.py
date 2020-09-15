#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Transform, Vector3, Quaternion, Point
from visualization_msgs.msg import InteractiveMarkerControl
from tf import TransformListener
from std_srvs.srv import Empty, EmptyRequest

from cgal_msgs.srv import FacetID
from cpt_utils.marker import RosMarker


class FindIdMarkerState:
    def __init__(self, initial_pose):
        self.pose = initial_pose

    def pose_changed(self, pose):
        if pose != self.pose:
            self.pose = pose
            return True
        return False

def loop(marker, state, get_id):
    if state.pose_changed(marker.marker.pose):
        position = marker.marker.pose.position
        current_point = Point(x=position.x, y=position.y, z=position.z)
        resp = get_id(current_point)
        marker.change_description('Primitive ID: {}'.format(resp.facet_id))


if __name__ == "__main__":
    rospy.init_node("find_id_in_mesh")
    marker = RosMarker('FindIDS',
                       'Move to search for ids.',
                       'mesh_id_marker',
                       InteractiveMarkerControl.MOVE_3D,
                       show_controls=True,
                       position=[1, 1, 1])
    state = FindIdMarkerState(marker.marker.pose)

    get_id = rospy.ServiceProxy('/mesh_publisher/get_triangle_id', FacetID)

    # Create a timer to regularly query for new IDs
    rospy.Timer(rospy.Duration(0.01), lambda msg: loop(
        marker, state, get_id))

    rospy.spin()
