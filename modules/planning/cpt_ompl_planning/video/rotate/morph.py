import open3d as o3d
import numpy as np
import copy

scenarios = [
    [
        "/home/mpantic/ws/rmp/src/cad-percept/modules/planning/cpt_ompl_planning/video/morph/curve3d.off",
        0.1,
        "curve"
        ],
    [
        "/home/mpantic/ws/rmp/src/cad-percept/modules/planning/cpt_ompl_planning/video/morph/hilo3d.off",
        0.45,
        "hilo"
        ],
    [
        "/home/mpantic/ws/rmp/src/cad-percept/modules/planning/cpt_ompl_planning/video/morph/rhone3d.off",
        0.01,
        "rhone"
        ]
]
scenario = scenarios[2]

mesh3d = o3d.io.read_triangle_mesh(
    scenario[0])
mesh3d.translate(-mesh3d.get_center())

mesh3d.scale(scenario[1], center=mesh3d.get_center())

mesh3d.compute_vertex_normals()


i_frame = 0


def rotate_view(vis):
    global slider, i_frame
    if i_frame == 0:
        vis.get_render_option().load_from_json("RenderOption_2020-10-09-21-18-00.json")
        ctr = vis.get_view_control()
        parameters = o3d.io.read_pinhole_camera_parameters("ScreenCamera_2020-10-15-18-20-10.json")
        ctr.convert_from_pinhole_camera_parameters(parameters)

    i_frame += 1
    if i_frame < 400:
        return True

    ctr = vis.get_view_control()
    ctr.rotate(5.0, 0.0)
    vis.capture_screen_image("imgs/" + scenario[2] + "/frame_" + str(i_frame-400) + ".png", True)
    return False



# vis = o3d.visualization.Visualizer()

vis = o3d.visualization.draw_geometries_with_animation_callback([mesh3d],
                                                                rotate_view)

vis.run()

vis.destroy_window()
