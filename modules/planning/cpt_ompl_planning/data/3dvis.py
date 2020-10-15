import open3d as o3d
from tabulate import tabulate

meshes  = {
"rhone": "/home/mpantic/ws/rmp/src/manifold_simulations/models/rhone_mesh/meshes/rhone_enu_0.1m.off",
"curve": "/home/mpantic/Work/RAL_Manifolds/curve.off",
"hilo": "/home/mpantic/ws/rmp/src/manifold_simulations/models/hilo_roof/meshes/hilo_reconstructed.off"
}

table = []
table.append(["", "Faces", "Extent [m]", "Surface Area [m^2]" ])

for scenario in meshes:
    path = meshes[scenario]
    mesh = o3d.io.read_triangle_mesh(path)
    #get bounding box extent
    extent = mesh.get_axis_aligned_bounding_box().get_extent()
    n_faces = len(mesh.triangles)
    area = mesh.get_surface_area()
    row = []
    row.append(scenario)
    row.append("{:.1f}k".format(n_faces/1000.0))
    row.append("{:.1f} \\times {:.1f} \\times {:.1f}".format(extent[0], extent[1], extent[2]))
    row.append("{:.1f}".format(area))

    table.append(row)


print(tabulate(table, tablefmt="latex_raw"))



exit(0)





#mesh = o3d.io.read_triangle_mesh("/home/mpantic/ws/rmp/src/manifold_simulations/models/rhone_mesh/meshes/rhone_enu_0.1m.off")
#mesh = o3d.io.read_triangle_mesh("/home/mpantic/Work/RAL_Manifolds/curve.off")
mesh = o3d.io.read_triangle_mesh("/home/mpantic/ws/rmp/src/manifold_simulations/models/hilo_roof/meshes/hilo_reconstructed.off")
mesh.compute_vertex_normals()
print(mesh.get_axis_aligned_bounding_box().get_extent())

wireframe = o3d.geometry.LineSet.create_from_triangle_mesh(mesh)
wireframe.paint_uniform_color([0, 0., 0])


vis = o3d.visualization.Visualizer()
vis.create_window()
vis.get_render_option().load_from_json("RenderOption_2020-10-09-21-18-00.json")
vis.update_renderer()
vis.add_geometry(mesh)
vis.add_geometry(mesh.get_oriented_bounding_box())
vis.add_geometry(wireframe)
vis.run()
vis.destroy_window()