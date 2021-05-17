import open3d as o3d
import numpy as np
import pandas as pd
from pandas import read_csv
import seaborn as sns


from random import randint


mesh = o3d.io.read_triangle_mesh("rhone3d.off")
mesh.compute_vertex_normals()
wireframe = o3d.geometry.LineSet.create_from_triangle_mesh(mesh)
wireframe.paint_uniform_color([0.4, 0.4, 0.4])


scenario = "rhone"
base_path = "/home/mpantic/ws/rmp/src/cad-percept/modules/planning/cpt_ompl_planning/evaluation/Okt12-1759_100/"


path_objects = []

def drawPath(vis, runid, radius=0.05, color=[1.0, 0.0, 0.0], skip=1):
    file_path = base_path + "paths/path_" + runid + ".log"
    try:
        points = np.genfromtxt(file_path, delimiter='\t')
    except:
        return


    for i in range(skip, len(points), skip):
        start = points[i - skip]
        end = points[i]
        segment_len = np.linalg.norm(end - start)
        if segment_len == 0:
            continue

        # construct rotation matrix
        R = np.eye(4)

        R[0:3, 2] = (end - start) / segment_len
        R[0:3, 1] = [R[1, 2], -R[0, 2], R[2, 2]]
        R[0:3, 0] = np.cross(R[0:3, 1], R[0:3, 2])
        R[0:3, 0] = R[0:3, 0] / np.linalg.norm(R[0:3, 0])

        mesh_cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=radius,
                                                                  height=segment_len)
        mesh_cylinder.compute_vertex_normals()
        mesh_cylinder.transform(R)
        mesh_cylinder.translate(start + (end-start)/2.0)
        mesh_cylinder.paint_uniform_color(color)
        path_objects.append(mesh_cylinder)
        vis.add_geometry(mesh_cylinder)


def read_files():
    run_id = {}
    df = read_csv(base_path+scenario+"_data.log", delimiter="\t",header=None)

    df.columns = ["scenario", "planner", "success", "duration", "length", "dist_surf", "smoothness",
                  "segments", "rowid", "runid"]
    palette = (sns.color_palette("Paired"))

    planners = [["RMP8", palette[5], 30, 1.0],
                ["DGEO", palette[3], 1,0.5],
                ["RRTConnect1000",  palette[11], 1,0.5],
                ["RRTStar1000", palette[1],1 ,0.5],
                ["RRTMeshProj1000",palette[7], 1,0.5],
                ["RRTStar250", palette[0], 1,0.5],
                ["RRTMeshProj250", palette[6], 1,0.5]]


    for planner, color, skip, width in planners:

        run_id[planner] = [(df[(df["planner"] == planner)]["runid"]).values, color, skip, width]

    return run_id

run_ids = read_files()

#exit(0)
i_frame = 0
i_frame_monotonic = 0
wireframe = o3d.geometry.LineSet.create_from_triangle_mesh(mesh)
wireframe.paint_uniform_color([0.4, 0.4, 0.4])

def rotate_view(vis):
    global  i_frame, i_frame_monotonic
    if i_frame == 0:
        vis.get_render_option().load_from_json("RenderOption_2020-10-09-21-18-00.json")

    i_frame += 1
    if i_frame % 25:
        ctr = vis.get_view_control()
        parameters = o3d.io.read_pinhole_camera_parameters("ScreenCamera_2020-10-15-20-57-47.json")
        ctr.convert_from_pinhole_camera_parameters(parameters)
        i_frame_monotonic+=1
        vis.capture_screen_image("imgs/"+scenario+"/frame_" + str(i_frame_monotonic) + ".png", True)
        return False

    selected = randint(0, 98)

    # dont save these frames - they make it shaky as it resets the cam
    for path_obj in path_objects:
        vis.remove_geometry(path_obj, False)
    path_objects.clear()

    for planner in run_ids:
        run_id =run_ids[planner][0][selected]
        drawPath(vis, run_id,run_ids[planner][3], run_ids[planner][1], run_ids[planner][2])
    return True


vis = o3d.visualization.draw_geometries_with_animation_callback([mesh],
                                                                rotate_view)

vis.run()

vis.destroy_window()

