import open3d as o3d
import numpy as np
import pandas as pd
from pandas import read_csv
import seaborn as sns
import sys



hilo_path="/home/mpantic/ws/rmp/src/manifold_simulations/models/hilo_roof/meshes/hilo_reconstructed.off"
rhone_path="/home/mpantic/ws/rmp/src/manifold_simulations/models/rhone_mesh/meshes/rhone_enu_0.1m.off"
curve_path="/home/mpantic/Work/RAL_Manifolds/curve.off"

path = hilo_path
mesh = o3d.io.read_triangle_mesh(path)
mesh.compute_vertex_normals()
wireframe = o3d.geometry.LineSet.create_from_triangle_mesh(mesh)
wireframe.paint_uniform_color([0.4, 0.4, 0.4])



scenario = "hilo"
base_path = "/home/mpantic/ws/rmp/src/cad-percept/modules/planning/cpt_ompl_planning/evaluation/Dez30-0958_100/"
#rowid = 27
rowid = int(sys.argv[1])

vis = o3d.visualization.Visualizer()
vis.create_window()
#vis.get_render_option().load_from_json("RenderOption_2020-10-09-21-18-00.json")
vis.update_renderer()

ctr = vis.get_view_control()
ctr.change_field_of_view(step=40)
print(ctr.get_field_of_view())


vis.add_geometry(wireframe)


def drawPath(vis, runid, radius=0.01, color=[1.0, 0.0, 0.0], skip=1):
    file_path = base_path + "paths/path_" + runid + ".log"
    points = np.genfromtxt(file_path, delimiter='\t')


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
        vis.add_geometry(mesh_cylinder)


def read_files():
    df = read_csv(base_path+scenario+"_data.log", delimiter="\t")
    df.columns = ["scenario", "planner", "success", "duration", "length", "dist_surf", "smoothness",
                  "segments", "rowid", "runid"]
    df = df[(df["rowid"] == rowid)]

    palette = (sns.color_palette("Paired"))

    planners = [["RMP8", palette[5], 10],
                ["DGEO", palette[3], 1],
		["CHOMP", palette[8], 1],
                ["RRTConnect1000",  palette[11], 1],
                ["RRTStar1000", palette[1],1 ],
                ["RRTMeshProj1000",palette[7], 1],
                ["RRTStar250", palette[0], 1],
                ["RRTMeshProj250", palette[6], 1],]


    for planner, color, skip in planners:
        success = (df[(df["planner"] == planner)]["success"].values[0])
        if not success:
            continue
        run_id = (df[(df["planner"] == planner)]["runid"].values[0])

        drawPath(vis, run_id, 0.025, np.array(color), skip)

read_files()

vis.run()
vis.destroy_window()
