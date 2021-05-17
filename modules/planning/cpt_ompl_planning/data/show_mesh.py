import open3d as o3d
import numpy as np
import pandas as pd
from pandas import read_csv
import seaborn as sns


hilo_path="/home/mpantic/ws/rmp/src/manifold_simulations/models/hilo_roof/meshes/hilo_reconstructed.off"
rhone_path="/home/mpantic/ws/rmp/src/manifold_simulations/models/rhone_mesh/meshes/rhone_enu_0.1m.off"
curve_path="/home/mpantic/Work/RAL_Manifolds/curve.off"

path = rhone_path
mesh = o3d.io.read_triangle_mesh(path)
mesh.compute_vertex_normals()
mesh.paint_uniform_color([50/255.0, 50/255.0, 50/255.0])



scenario = "hilo"
base_path = "/home/mpantic/ws/rmp/src/cad-percept/modules/planning/cpt_ompl_planning/evaluation/Okt12-1759_100/"
#rowid = 27
rowid = 88
i_frame = 0
def rotate_view(vis):
    global i_frame
    if i_frame==0:
        vis.get_render_option().load_from_json("RenderOption_2020-10-09-21-18-00.json")
    i_frame+=1
    vis.capture_screen_image( "imgs" + "/frame_" + str(i_frame) + ".png", True)

vis = o3d.visualization.draw_geometries_with_animation_callback([mesh],
                                                                rotate_view)
