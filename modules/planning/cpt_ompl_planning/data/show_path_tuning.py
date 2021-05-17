import open3d as o3d
import numpy as np
import pandas as pd
from pandas import read_csv
import seaborn as sns

import glob


import numpy as np
import open3d as o3d


def align_vector_to_another(a=np.array([0, 0, 1]), b=np.array([1, 0, 0])):
    """
    Aligns vector a to vector b with axis angle rotation
    """
    if np.array_equal(a, b):
        return None, None
    axis_ = np.cross(a, b)
    if np.linalg.norm(axis_) < 0.0001:
        return None, None
    axis_ = axis_ / np.linalg.norm(axis_)
    angle = np.arccos(np.dot(a, b))

    return axis_, angle


def normalized(a, axis=-1, order=2):
    """Normalizes a numpy array of points"""
    l2 = np.atleast_1d(np.linalg.norm(a, order, axis))
    l2[l2 == 0] = 1
    return a / np.expand_dims(l2, axis), l2


class LineMesh(object):
    def __init__(self, points, lines=None, colors=[0, 1, 0], radius=0.15):
        """Creates a line represented as sequence of cylinder triangular meshes

        Arguments:
            points {ndarray} -- Numpy array of ponts Nx3.

        Keyword Arguments:
            lines {list[list] or None} -- List of point index pairs denoting line segments. If None, implicit lines from ordered pairwise points. (default: {None})
            colors {list} -- list of colors, or single color of the line (default: {[0, 1, 0]})
            radius {float} -- radius of cylinder (default: {0.15})
        """
        self.points = np.array(points)
        self.lines = np.array(
            lines) if lines is not None else self.lines_from_ordered_points(self.points)
        self.colors = np.array(colors)
        self.radius = radius
        self.cylinder_segments = []

        self.create_line_mesh()

    @staticmethod
    def lines_from_ordered_points(points):
        lines = [[i, i + 1] for i in range(0, points.shape[0] - 1, 1)]
        return np.array(lines)

    def create_line_mesh(self):
        first_points = self.points[self.lines[:, 0], :]
        second_points = self.points[self.lines[:, 1], :]
        line_segments = second_points - first_points
        line_segments_unit, line_lengths = normalized(line_segments)

        z_axis = np.array([0, 0, 1])
        # Create triangular mesh cylinder segments of line
        for i in range(line_segments_unit.shape[0]):
            line_segment = line_segments_unit[i, :]
            line_length = line_lengths[i]
            # get axis angle rotation to allign cylinder with line segment
            axis, angle = align_vector_to_another(z_axis, line_segment)
            # Get translation vector
            translation = first_points[i, :] + line_segment * line_length * 0.5
            # create cylinder and apply transformations
            cylinder_segment = o3d.geometry.TriangleMesh.create_cylinder(
                self.radius, line_length)
            cylinder_segment = cylinder_segment.translate(
                translation, relative=False)
            if axis is not None:
                axis_a = axis * angle
                R=o3d.geometry.get_rotation_matrix_from_axis_angle(axis_a)

                cylinder_segment = cylinder_segment.rotate(
                    np.array(R), center=cylinder_segment.get_center())
                #cylinder_segment = cylinder_segment.rotate(
                #   axis_a, center=True, type=o3d.geometry.RotationType.AxisAngle)
            # color cylinder
            color = self.colors if self.colors.ndim == 1 else self.colors[i, :]
            cylinder_segment.paint_uniform_color(color)

            self.cylinder_segments.append(cylinder_segment)

    def add_line(self, vis):
        """Adds this line to the visualizer"""
        for cylinder in self.cylinder_segments:
            vis.add_geometry(cylinder)

    def remove_line(self, vis):
        """Removes this line from the visualizer"""
        for cylinder in self.cylinder_segments:
            vis.remove_geometry(cylinder)



if __name__ == "__main__":

    hilo_path = "/home/mpantic/ws/rmp/src/manifold_simulations/models/hilo_roof/meshes/hilo_reconstructed.off"

    path = hilo_path
    mesh = o3d.io.read_triangle_mesh(path)
    mesh.compute_vertex_normals()
    wireframe = o3d.geometry.LineSet.create_from_triangle_mesh(mesh)
    wireframe.paint_uniform_color([0.4, 0.4, 0.4])

    scenario = "hilo"
    base_path = "/home/mpantic/ws/rmp/src/cad-percept/modules/planning/cpt_ompl_planning/evaluation/param_eval/"
# rowid = 27
    rowid = 88

    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.get_render_option().load_from_json("RenderOption_2020-10-09-21-18-00.json")
    vis.update_renderer()

    ctr = vis.get_view_control()
    ctr.change_field_of_view(step=40)
    print(ctr.get_field_of_view())

    vis.add_geometry(mesh)


    def drawPath(vis, file_path, radius=0.05, color=[1.0, 0.0, 0.0], skip=1):
        print(sns.light_palette("seagreen", as_cmap=True))

        colors = {
            "c56b01f9-9ed7-4bc5-a826-e42c0ec7203e": [0.0, 0.0, 1.0],
            # A: along mesh
           "2a31f2c1-ee3f-4d61-82a2-f953e495a9e7": [0.1, 0.0, 0.0],
           "bdd98e23-4595-4651-895c-ad8838253400": [0.2, 0.0, 0.0],
           "726d61fb-b33b-4acb-81fb-7c0d003bb088": [0.3, 0.0, 0.0],
           "c85d2b12-e79c-41ed-b794-5fe42847686c": [0.4, 0.0, 0.0],
           "da05c232-a76b-4498-b003-81c23a5cd514": [0.5, 0.0, 0.0],
           "b8b0dcda-c7df-46e7-ae3b-e043dc498174": [0.6, 0.0, 0.0],
           "b45d0356-4c78-4dd5-bef0-b8873f7777d1": [0.7, 0.0, 0.0],


            #"a1215402-e390-43c0-b98b-d1139996abd8": [0.0, 0.2, 0.0],
            #"7c6b9dfc-edf8-4243-ba35-800070818736": [0.0, 0.3, 0.0],
            #"ec977500-cdb8-447d-8389-e84cfd0e6b5d": [0.0, 0.4, 0.0],
            #"dc0fd80b-3756-445d-a3bf-0349e959f060": [0.0, 0.5, 0.0],
            #"aed84ced-5e59-4161-b494-76935f6e6670": [0.0, 0.6, 0.0],
            #"a782e9cb-04f6-4810-a645-116be1b31e71": [0.0, 0.7, 0.0],
            #"38b927ea-b605-4962-a0bb-b418bac45000": [0.0, 0.8, 0.0]
        }
        selectedcolor = [0,0,0]

        #horrible... no time to do proper now.
        for id in colors:
            if id in file_path:
                print(id)
                print(colors)
                selectedcolor = colors[id]
                break

        if selectedcolor == [0,0,0]: return

        points = np.genfromtxt(file_path, delimiter='\t')
        line_ix = np.vstack([np.arange(0, len(points) - 1), np.arange(1, len(points))]).T
# o3d.utility.Vector2iVector(line_ix)
        lines = LineMesh(o3d.utility.Vector3dVector(points))
#        lines.points = o3d.utility.Vector3dVector(points)

#        lines.lines = o3d.utility.Vector2iVector(line_ix)
#        print(selectedcolor)
#        lines.paint_uniform_color(selectedcolor)
        line_mesh1_geoms = lines.cylinder_segments
        for seg in lines.cylinder_segments:
            vis.add_geometry(seg)


    for f in glob.glob(
            "/home/mpantic/ws/rmp/src/cad-percept/modules/planning/cpt_ompl_planning/evaluation/param_eval/path*"):
        drawPath(vis, f, 0.5, [1.0, 0.0, 0.0], 1)

    mesh_sphere_start = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
    mesh_sphere_start.compute_vertex_normals()
    mesh_sphere_start.paint_uniform_color([0.0, 0.8, 0.0])
    mesh_sphere_start.translate([9.63312, 5.56065, 7])
    vis.add_geometry(mesh_sphere_start)

    mesh_sphere_end = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
    mesh_sphere_end.compute_vertex_normals()
    mesh_sphere_end.paint_uniform_color([0.8, 0.0, 0.0])
    mesh_sphere_end.translate([9.37443, 2.55775, 3.22736])
    vis.add_geometry(mesh_sphere_end)

    vis.run()
    vis.destroy_window()
