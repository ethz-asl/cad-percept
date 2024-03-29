import rospy
import numpy as np
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerControl
from geometry_msgs.msg import Point, Quaternion


def makeBox(msg, scale=0.35, color=[0.5, 0.5, 0.5, 1]):
    """
    Produces the marker box.

    Args:
        scale: float, proportional to the size of the box
        color: List of 4 floats, RGBA color
    Returns:
        marker
    """
    assert len(color) == 4

    marker = Marker()
    marker.type = Marker.CUBE
    marker.scale.x = msg.scale * scale
    marker.scale.y = msg.scale * scale
    marker.scale.z = msg.scale * scale
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]
    return marker


def makeBoxControl(marker_msg):
    control = InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append(makeBox(marker_msg))
    marker_msg.controls.append(control)
    return control


def normalized_quaternion(x, y, z, w):
    """
    Creates a normalized Quaternion.
    """
    s = 1.0 / np.linalg.norm([x, y, z, w])
    ret = Quaternion()
    ret.x = s * x
    ret.y = s * y
    ret.z = s * z
    ret.w = s * w
    return ret


def make_control(orientation, name, rotation=False, fixed=False):
    """
    Creates an InteractiveMarkerControl.

    Args:
        orientation: List [x, y, z, w]
    """
    control = InteractiveMarkerControl()
    control.orientation = normalized_quaternion(*orientation)
    control.name = name
    if rotation:
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    else:
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    if fixed:
        control.orientation_mode = InteractiveMarkerControl.FIXED
    return control


class RosMarker:
    def __init__(self, name, description, server_name, interaction_mode,
                 position=[0, 0, 0], orientation=[0, 0, 1, 1], parent_frame=None,
                 fixed=False, show_controls=False):
        marker = InteractiveMarker()
        parent_frame = parent_frame or rospy.get_param('~marker_parent_frame')
        marker.header.frame_id = parent_frame
        marker.pose.position = Point(*position)
        marker.pose.orientation = normalized_quaternion(*orientation)
        marker.scale = 1

        marker.name = name
        marker.description = description

        # insert a box
        makeBoxControl(marker)
        marker.controls[0].interaction_mode = interaction_mode

        if show_controls:
            if interaction_mode in [InteractiveMarkerControl.ROTATE_3D,
                                    InteractiveMarkerControl.MOVE_ROTATE_3D]:
                # rotation controls
                marker.controls.append(make_control(
                    [1, 0, 0, 1], 'rotate_x', rotation=True, fixed=fixed))
                marker.controls.append(make_control(
                    [0, 1, 0, 1], 'rotate_y', rotation=True, fixed=fixed))
                marker.controls.append(make_control(
                    [0, 0, 1, 1], 'rotate_z', rotation=True, fixed=fixed))

            if interaction_mode in [InteractiveMarkerControl.MOVE_3D,
                                    InteractiveMarkerControl.MOVE_ROTATE_3D]:
                # translation controls
                marker.controls.append(make_control(
                    [1, 0, 0, 1], 'move_x', rotation=False, fixed=fixed))
                marker.controls.append(make_control(
                    [0, 1, 0, 1], 'move_y', rotation=False, fixed=fixed))
                marker.controls.append(make_control(
                    [0, 0, 1, 1], 'move_z', rotation=False, fixed=fixed))

        self.marker = marker
        self.server = InteractiveMarkerServer(server_name)
        self.server.insert(self.marker, self.processFeedback)
        self.menu_handler = MenuHandler()
        self.menu_handler.apply(self.server, self.marker.name)
        self.server.applyChanges()

    def processFeedback(self, feedback):
        self.server.applyChanges()

    def add_menu_item(self, name, callback):
        self.menu_handler.insert(name, callback=callback)
        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

    def change_description(self, new_description):
        self.marker.description = new_description
        self.server.insert(self.marker, self.processFeedback)


if __name__ == "__main__":
    rospy.init_node("marker_example")

    marker = RosMarker('test marker', 'test marker description', 'test_marker_server',
                       InteractiveMarkerControl.MOVE_3D, [3, 0, 0])
    rospy.spin()
