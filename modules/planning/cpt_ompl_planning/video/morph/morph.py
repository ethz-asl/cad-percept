import open3d as o3d
import numpy as np
import copy

scenarios = [
    [
        "/home/mpantic/ws/rmp/src/cad-percept/modules/planning/cpt_ompl_planning/video/morph/curve2d.off",
        "/home/mpantic/ws/rmp/src/cad-percept/modules/planning/cpt_ompl_planning/video/morph/curve3d.off",
        0.15,
        4.0,
        0,
        0,
        0,
        np.pi,
        "curve"
        ],
    [
        "/home/mpantic/ws/rmp/src/cad-percept/modules/planning/cpt_ompl_planning/video/morph/hilo2d.off",
        "/home/mpantic/ws/rmp/src/cad-percept/modules/planning/cpt_ompl_planning/video/morph/hilo3d.off",
        0.45,
        4.0,
        -1.0,
        1.0,
        0,
        np.pi,
        "hilo"
        ],
    [
        "/home/mpantic/ws/rmp/src/cad-percept/modules/planning/cpt_ompl_planning/video/morph/rhone2d.off",
        "/home/mpantic/ws/rmp/src/cad-percept/modules/planning/cpt_ompl_planning/video/morph/rhone3d.off",
        0.08,
        4.0,
        0.0,
        0.0,
        np.pi / 2,
        np.pi / 2,
        "rhone"
        ]
]
scenario = scenarios[2]
mesh2d = o3d.io.read_triangle_mesh(
    scenario[0])
mesh3d = o3d.io.read_triangle_mesh(
    scenario[1])

mesh3d.scale(scenario[2], center=mesh3d.get_center())
mesh2d.scale(scenario[3], center=mesh2d.get_center())

mesh3d.translate(-mesh3d.get_center() + [0, scenario[4], scenario[5]])
mesh2d.translate(-mesh2d.get_center())

R = mesh3d.get_rotation_matrix_from_xyz((0, 0, scenario[6]))
mesh3d.rotate(R, center=(0, 0, 0))

R = mesh2d.get_rotation_matrix_from_xyz((0, 0, scenario[7]))
mesh2d.rotate(R, center=(0, 0, 0))

slider = 0.0
vertices_2d_np = copy.deepcopy(np.asarray(mesh2d.vertices))
vertices_3d_np = copy.deepcopy(np.asarray(mesh3d.vertices))

i_frame = 0


def rotate_view(vis):
    global slider, i_frame
    if i_frame == 0:
        vis.get_render_option().load_from_json("renderoptions.json")
        ctr = vis.get_view_control()
        parameters = o3d.io.read_pinhole_camera_parameters("ScreenCamera_2020-10-15-18-20-10.json")
        ctr.convert_from_pinhole_camera_parameters(parameters)

    i_frame += 1
    if i_frame < 50:
        return True
    if slider < 1.0:
        slider += 0.01
    print(slider)
    vertices_interp = (1 - slider) * vertices_3d_np + slider * vertices_2d_np
    mesh3d.vertices = o3d.utility.Vector3dVector(vertices_interp)
    mesh3d.paint_uniform_color([1, 1, 1])

    vis.capture_screen_image("imgs/" + scenario[8] + "/frame_" + str(i_frame-50) + ".png", True)
    return True


# vis = o3d.visualization.Visualizer()

vis = o3d.visualization.draw_geometries_with_animation_callback([mesh3d],
                                                                rotate_view)

vis.run()

vis.destroy_window()
