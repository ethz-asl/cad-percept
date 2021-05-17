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

    start_pos = points[0]
    end_pos = points[len(points) - 1]

    mesh_cylinder_s = o3d.geometry.TriangleMesh.create_cylinder(radius=0.25,
                                                              height=0.25)
    mesh_cylinder_s.compute_vertex_normals()
    mesh_cylinder_s.translate(start_pos)
    mesh_cylinder_s.paint_uniform_color([0.0, 1.0, 0.0])
    vis.add_geometry(mesh_cylinder_s)

    mesh_cylinder_e = o3d.geometry.TriangleMesh.create_cylinder(radius=0.25,
                                                                height=0.25)
    mesh_cylinder_e.compute_vertex_normals()
    mesh_cylinder_e.translate(end_pos)
    mesh_cylinder_e.paint_uniform_color([1.0, 0.0, 0.0])
    vis.add_geometry(mesh_cylinder_e)


def read_files():
    for rowid in range(0,99):
        df = read_csv(base_path+scenario+"_data.log", delimiter="\t")
        df.columns = ["scenario", "planner", "success", "duration", "length", "dist_surf", "smoothness",
                      "segments", "rowid", "runid"]
        df = df[(df["rowid"] == rowid)]

        palette = (sns.color_palette("Paired"))

        planners = [["DGEO", palette[3], 1]]


        for planner, color, skip in planners:
            success = (df[(df["planner"] == planner)]["success"].values[0])
            if not success:
                continue
            run_id = (df[(df["planner"] == planner)]["runid"].values[0])

            drawPath(vis, run_id, 0.02, np.array(color), skip)

read_files()

vis.run()
vis.destroy_window()
