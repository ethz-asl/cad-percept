import open3d as o3d
import numpy as np
import pandas as pd
from pandas import read_csv
import seaborn as sns
import colorsys

hilo_path = "hilo2d.off"
transl = np.array([0.999797, 0.485739, -0.001])


def get2field(i):
    field2d = np.genfromtxt("2d/" + str(frame) + ".csv", delimiter="\t")
    vector_length = np.sqrt(np.sum(np.power(field2d[:, 3:6],2),1))
    colors = []
    for x in range(0, len(vector_length)):
        field2d[x, 3:6] =  0.1*field2d[x, 3:6] #/np.linalg.norm( field2d[x, 3:5])
        color_angle = np.arctan2(field2d[x, 3], field2d[x, 4])
        colors.append(colorsys.hsv_to_rgb(color_angle, 1.0, 1.0))

    field2dpoints = np.vstack([field2d[:, 0:3], field2d[:, 0:3] + field2d[:, 3:6]])
    field2dpoints[:,2] = 0
    acc_field = o3d.geometry.LineSet()
    acc_field.points = o3d.utility.Vector3dVector(field2dpoints)
    lines_np = np.vstack(
        [np.arange(0, len(field2d)), np.arange(len(field2d), len(field2dpoints))]).T
    acc_field.colors = o3d.utility.Vector3dVector(colors)
    acc_field.lines = o3d.utility.Vector2iVector(lines_np)
    return acc_field


mesh2d = o3d.io.read_triangle_mesh(hilo_path)
mesh2d.compute_vertex_normals()
mesh2d.translate(transl)
wireframe = o3d.geometry.LineSet.create_from_triangle_mesh(mesh2d)
wireframe.paint_uniform_color([0, 0., 0])
frame = 0
#old_field = get2field(0)

def rotate_view(vis):
    global frame
    #global frame, old_field
    #new_field = get2field(frame)
    #vis.remove_geometry(old_field, False)
    #vis.add_geometry(new_field)
    #old_field = new_field

    positions = np.genfromtxt("pos/" + str(frame) + ".csv", delimiter="\t")

    pos2d = positions[ 3:6] + transl
    pos2d[2] = 0
    print(pos2d)

    mesh_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
    mesh_sphere.compute_vertex_normals()
    mesh_sphere.paint_uniform_color([0.1, 0.1, 0.7])
    mesh_sphere.translate(pos2d)

    frame += 1
    vis.add_geometry(mesh_sphere, False)
    vis.capture_screen_image( "2d_img" + "/frame_" + str(frame) + ".png", True)


    return False


vis = o3d.visualization.draw_geometries_with_animation_callback([wireframe],
                                                                rotate_view)
