#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Transform, Vector3, Quaternion
from visualization_msgs.msg import InteractiveMarkerControl
from tf import TransformListener
from std_srvs.srv import Empty, EmptyRequest

from cgal_msgs.srv import PublishMesh
from cpt_utils.marker import RosMarker


class FindIdMarkerState:
    def __init__(self, initial_pose):
        self.pose = initial_pose

    def pose_changed(self, pose):
        if pose != self.pose:
            self.pose = pose
            return True
        return False

def loop(marker, state):
    if state.pose_changed(marker.marker.pose):
        marker.change_description('pose changed')


if __name__ == "__main__":
    rospy.init_node("find_id_in_mesh")
    marker = RosMarker('FindIDS',
                       'description',
                       'mesh_id_marker',
                       InteractiveMarkerControl.MOVE_3D,
                       show_6dof=False,
                       position=[1, 1, 1],
                       parent_frame='marker_position')
    state = FindIdMarkerState(marker.marker.pose)

    publish_mesh = rospy.ServiceProxy('/mesh_publisher/publish', PublishMesh)

    # Create a timer to regularly query for new IDs
    rospy.Timer(rospy.Duration(0.01), lambda msg: loop(
        marker, state))

    rospy.spin()
