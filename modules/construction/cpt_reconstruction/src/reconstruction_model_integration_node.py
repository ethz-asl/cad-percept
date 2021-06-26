#!/usr/bin/env python

import open3d as o3d

import rospy
from cpt_reconstruction.msg import element_proposals, parameters
from geometry_msgs.msg import Vector3

import numpy as np

from plyfile import PlyData, PlyElement

import ttk
import Tkinter as tk

manipulationPanel = None
BUILDING_MODEL_PATH_ = ""


class ManipulationPanel:
    def __init__(self):
        self.root = tk.Tk()
        self.root.protocol("WM_DELETE_WINDOW", self.callback)

        self.root.title("Manipulation Panel")

        self.label_planar_element = tk.Label(master=self.root, text='Planar Element: ')
        self.label_planar_element.grid(row=0, column=0, padx='5', pady='5', sticky='ew')

        self.planar_parameters = tk.Label(master=self.root, text='Parameters [m]: ')
        self.planar_parameters.grid(row=1, column=1, padx='10', pady='5', sticky='ew')

        self.label_a1 = tk.Label(master=self.root, text='a1: ', anchor='e')
        self.variable_a1 = tk.DoubleVar(value=0)
        self.spinbox_a1 = tk.Spinbox(self.root, from_=-100, to=100, increment=0.05, width=15, justify=tk.LEFT,
                                     textvariable=self.variable_a1)

        self.label_a1.grid(row=2, column=1, padx='5', pady='5', sticky='ew')
        self.spinbox_a1.grid(row=2, column=2, padx='5', pady='5', sticky='ew')

        self.label_a2 = tk.Label(master=self.root, text='a2: ', anchor='e')
        self.variable_a2 = tk.DoubleVar(value=0)
        self.spinbox_a2 = tk.Spinbox(self.root, from_=-100, to=100, increment=0.05, width=15, justify=tk.LEFT,
                                     textvariable=self.variable_a2)
        self.label_a2.grid(row=2, column=3, padx='5', pady='5', sticky='ew')
        self.spinbox_a2.grid(row=2, column=4, padx='5', pady='5', sticky='ew')

        self.label_b1 = tk.Label(master=self.root, text='b1: ', anchor='e')
        self.variable_b1 = tk.DoubleVar(value=0)
        self.spinbox_b1 = tk.Spinbox(self.root, from_=-100, to=100, increment=0.05, width=15, justify=tk.LEFT,
                                     textvariable=self.variable_b1)
        self.label_b1.grid(row=3, column=1, padx='5', pady='5', sticky='ew')
        self.spinbox_b1.grid(row=3, column=2, padx='5', pady='5', sticky='ew')

        self.label_b2 = tk.Label(master=self.root, text='b2: ', anchor='e')
        self.variable_b2 = tk.DoubleVar(value=0)
        self.spinbox_b2 = tk.Spinbox(self.root, from_=-100, to=100, increment=0.05, width=15, justify=tk.LEFT,
                                     textvariable=self.variable_b2)
        self.label_b2.grid(row=3, column=3, padx='5', pady='5', sticky='ew')
        self.spinbox_b2.grid(row=3, column=4, padx='5', pady='5', sticky='ew')

        self.label_c1 = tk.Label(master=self.root, text='c1: ', anchor='e')
        self.variable_c1 = tk.DoubleVar(value=0)
        self.spinbox_c1 = tk.Spinbox(self.root, from_=-100, to=100, increment=0.05, width=15, justify=tk.LEFT,
                                     textvariable=self.variable_c1)
        self.label_c1.grid(row=4, column=1, padx='5', pady='5', sticky='ew')
        self.spinbox_c1.grid(row=4, column=2, padx='5', pady='5', sticky='ew')

        self.label_c2 = tk.Label(master=self.root, text='c2: ', anchor='e')
        self.variable_c2 = tk.DoubleVar(value=0)
        self.spinbox_c2 = tk.Spinbox(self.root, from_=-100, to=100, increment=0.05, width=15, justify=tk.LEFT,
                                     textvariable=self.variable_c2)
        self.label_c2.grid(row=4, column=3, padx='5', pady='5', sticky='ew')
        self.spinbox_c2.grid(row=4, column=4, padx='5', pady='5', sticky='ew')

        # Translation
        self.planar_translation = tk.Label(master=self.root, text='Translation [m]: ')
        self.planar_translation.grid(row=5, column=1, padx='10', pady='5', sticky='ew')

        self.label_translation = tk.Label(master=self.root, text='(dx, dy, dz)', anchor='e')
        self.label_translation.grid(row=6, column=1, padx='20', pady='5', sticky='ew')

        self.variable_dx = tk.DoubleVar(value=0)
        self.spinbox_dx = tk.Spinbox(self.root, from_=-100, to=100, increment=0.05, width=15, justify=tk.LEFT,
                                     textvariable=self.variable_dx)
        self.spinbox_dx.grid(row=6, column=2, padx='5', pady='20', sticky='ew')

        self.variable_dy = tk.DoubleVar(value=0)
        self.spinbox_dy = tk.Spinbox(self.root, from_=-100, to=100, increment=0.05, width=15, justify=tk.LEFT,
                                     textvariable=self.variable_dy)
        self.spinbox_dy.grid(row=6, column=3, padx='5', pady='20', sticky='ew')

        self.variable_dz = tk.DoubleVar(value=0)
        self.spinbox_dz = tk.Spinbox(self.root, from_=-100, to=100, increment=0.05, width=15, justify=tk.LEFT,
                                     textvariable=self.variable_dz)
        self.spinbox_dz.grid(row=6, column=4, padx='5', pady='20', sticky='ew')

        # Rotation
        self.planar_rotation = tk.Label(master=self.root, text='Rotation [deg]: ')
        self.planar_rotation.grid(row=7, column=1, padx='10', pady='5', sticky='ew')

        self.label_rotation = tk.Label(master=self.root, text=(u"(d\u03b1, d\u03b2, d\u03b3)"), anchor='e')
        self.label_rotation.grid(row=8, column=1, padx='20', pady='5', sticky='ew')

        self.variable_rx = tk.DoubleVar(value=0)
        self.spinbox_rx = tk.Spinbox(self.root, from_=-1000, to=1000, increment=1.0, width=15, justify=tk.LEFT,
                                     textvariable=self.variable_rx)
        self.spinbox_rx.grid(row=8, column=2, padx='5', pady='20', sticky='ew')

        self.variable_ry = tk.DoubleVar(value=0)
        self.spinbox_ry = tk.Spinbox(self.root, from_=-1000, to=1000, increment=1.0, width=15, justify=tk.LEFT,
                                     textvariable=self.variable_ry)
        self.spinbox_ry.grid(row=8, column=3, padx='5', pady='20', sticky='ew')

        self.variable_rz = tk.DoubleVar(value=0)
        self.spinbox_rz = tk.Spinbox(self.root, from_=-1000, to=1000, increment=1.0, width=15, justify=tk.LEFT,
                                     textvariable=self.variable_rz)
        self.spinbox_rz.grid(row=8, column=4, padx='5', pady='20', sticky='ew')

        self.separator = ttk.Separator(self.root, orient='horizontal')
        self.separator.grid(row=9, column=0, columnspan=5, padx='5', pady='20', sticky='ew')

        # Cylinders
        self.label_cylider = tk.Label(master=self.root, text='Cylinder: ')
        self.label_cylider.grid(row=10, column=0, padx='5', pady='10', sticky='ew')

        self.lable_radius = tk.Label(master=self.root, text='Radius [m]: ')
        self.lable_radius.grid(row=11, column=1, padx='5', pady='10', sticky='ew')

        self.lable_r = tk.Label(master=self.root, text='r: ', anchor='e')
        self.lable_r.grid(row=12, column=1, padx='5', pady='10', sticky='ew')

        self.variable_radius = tk.DoubleVar(value=0)
        self.spinbox_radius = tk.Spinbox(self.root, from_=-100, to=100, increment=0.05, width=15, justify=tk.LEFT,
                                         textvariable=self.variable_radius)
        self.spinbox_radius.grid(row=12, column=2, padx='5', pady='20', sticky='ew')

        self.label_height = tk.Label(master=self.root, text='Height [m]: ')
        self.label_height.grid(row=13, column=1, padx='5', pady='10', sticky='ew')

        self.label_height1 = tk.Label(master=self.root, text='h1: ', anchor='e')
        self.label_height1.grid(row=14, column=1, padx='5', pady='10', sticky='ew')

        self.variable_h1 = tk.DoubleVar(value=0)
        self.spinbox_h1 = tk.Spinbox(self.root, from_=-100, to=100, increment=0.05, width=15, justify=tk.LEFT,
                                     textvariable=self.variable_h1)
        self.spinbox_h1.grid(row=14, column=2, padx='5', pady='20', sticky='ew')

        self.label_height2 = tk.Label(master=self.root, text='h2: ', anchor='e')
        self.label_height2.grid(row=14, column=3, padx='5', pady='10', sticky='ew')

        self.variable_h2 = tk.DoubleVar(value=0)
        self.spinbox_h2 = tk.Spinbox(self.root, from_=-100, to=100, increment=0.05, width=15, justify=tk.LEFT,
                                     textvariable=self.variable_h2)
        self.spinbox_h2.grid(row=14, column=4, padx='5', pady='20', sticky='ew')

        # Translation
        self.cyl_translation = tk.Label(master=self.root, text='Translation [m]: ')
        self.cyl_translation.grid(row=15, column=1, padx='10', pady='5', sticky='ew')

        self.cyl_label_translation = tk.Label(master=self.root, text='(dx, dy, dz)', anchor='e')
        self.cyl_label_translation.grid(row=16, column=1, padx='20', pady='5', sticky='ew')

        self.variable_cyl_dx = tk.DoubleVar(value=0)
        self.spinbox_cyl_dx = tk.Spinbox(self.root, from_=-100, to=100, increment=0.05, width=15, justify=tk.LEFT,
                                     textvariable=self.variable_cyl_dx)
        self.spinbox_cyl_dx.grid(row=16, column=2, padx='5', pady='20', sticky='ew')

        self.variable_cyl_dy = tk.DoubleVar(value=0)
        self.spinbox_cyl_dy = tk.Spinbox(self.root, from_=-100, to=100, increment=0.05, width=15, justify=tk.LEFT,
                                     textvariable=self.variable_cyl_dy)
        self.spinbox_cyl_dy.grid(row=16, column=3, padx='5', pady='20', sticky='ew')

        self.variable_cyl_dz = tk.DoubleVar(value=0)
        self.spinbox_cyl_dz = tk.Spinbox(self.root, from_=-100, to=100, increment=0.05, width=15, justify=tk.LEFT,
                                     textvariable=self.variable_cyl_dz)
        self.spinbox_cyl_dz.grid(row=16, column=4, padx='5', pady='20', sticky='ew')

        # Rotation
        self.cyl_rotation = tk.Label(master=self.root, text='Rotation [deg]: ')
        self.cyl_rotation.grid(row=17, column=1, padx='10', pady='5', sticky='ew')

        self.label_cyl_rotation = tk.Label(master=self.root, text=(u"(d\u03b1, d\u03b2, d\u03b3)"), anchor='e')
        self.label_cyl_rotation.grid(row=18, column=1, padx='20', pady='5', sticky='ew')

        self.variable_cyl_rx = tk.DoubleVar(value=0)
        self.spinbox_cyl_rx = tk.Spinbox(self.root, from_=-1000, to=1000, increment=1.0, width=15, justify=tk.LEFT,
                                     textvariable=self.variable_cyl_rx)
        self.spinbox_cyl_rx.grid(row=18, column=2, padx='5', pady='20', sticky='ew')

        self.variable_cyl_ry = tk.DoubleVar(value=0)
        self.spinbox_cyl_ry = tk.Spinbox(self.root, from_=-1000, to=1000, increment=1.0, width=15, justify=tk.LEFT,
                                     textvariable=self.variable_cyl_ry)
        self.spinbox_cyl_ry.grid(row=18, column=3, padx='5', pady='20', sticky='ew')

        self.variable_cyl_rz = tk.DoubleVar(value=0)
        self.spinbox_cyl_rz = tk.Spinbox(self.root, from_=-1000, to=1000, increment=1.0, width=15, justify=tk.LEFT,
                                     textvariable=self.variable_cyl_rz)
        self.spinbox_cyl_rz.grid(row=18, column=4, padx='5', pady='20', sticky='ew')


    def startMainloop(self):
        self.root.mainloop()

    def callback(self):
        self.root.quit()

    def destroy(self):
        self.root.destroy()

    def quit(self):
        self.root.quit()

    def activatePlanarElements(self):
        self.spinbox_a1["state"] = "normal"
        self.spinbox_a2["state"] = "normal"
        self.spinbox_b1["state"] = "normal"
        self.spinbox_b2["state"] = "normal"
        self.spinbox_c1["state"] = "normal"
        self.spinbox_c2["state"] = "normal"

        self.spinbox_dx["state"] = "normal"
        self.spinbox_dy["state"] = "normal"
        self.spinbox_dz["state"] = "normal"

        self.spinbox_rx["state"] = "normal"
        self.spinbox_ry["state"] = "normal"
        self.spinbox_rz["state"] = "normal"

    def deactivatePlanarElements(self):
        self.spinbox_a1["state"] = "disabled"
        self.spinbox_a2["state"] = "disabled"
        self.spinbox_b1["state"] = "disabled"
        self.spinbox_b2["state"] = "disabled"
        self.spinbox_c1["state"] = "disabled"
        self.spinbox_c2["state"] = "disabled"

        self.spinbox_dx["state"] = "disabled"
        self.spinbox_dy["state"] = "disabled"
        self.spinbox_dz["state"] = "disabled"

        self.spinbox_rx["state"] = "disabled"
        self.spinbox_ry["state"] = "disabled"
        self.spinbox_rz["state"] = "disabled"

    def activateCylinders(self):
        self.spinbox_radius["state"] = "normal"
        self.spinbox_h1["state"] = "normal"
        self.spinbox_h2["state"] = "normal"

        self.spinbox_cyl_dx["state"] = "normal"
        self.spinbox_cyl_dy["state"] = "normal"
        self.spinbox_cyl_dz["state"] = "normal"

        self.spinbox_cyl_rx["state"] = "normal"
        self.spinbox_cyl_ry["state"] = "normal"
        self.spinbox_cyl_rz["state"] = "normal"

    def deactivateCylinders(self):
        self.spinbox_radius["state"] = "disabled"
        self.spinbox_h1["state"] = "disabled"
        self.spinbox_h2["state"] = "disabled"

        self.spinbox_cyl_dx["state"] = "disabled"
        self.spinbox_cyl_dy["state"] = "disabled"
        self.spinbox_cyl_dz["state"] = "disabled"

        self.spinbox_cyl_rx["state"] = "disabled"
        self.spinbox_cyl_ry["state"] = "disabled"
        self.spinbox_cyl_rz["state"] = "disabled"

    def getA1Value(self):
        return float(self.variable_a1.get())

    def getA2Value(self):
        return float(self.variable_a2.get())

    def getB1Value(self):
        return float(self.variable_b1.get())

    def getB2Value(self):
        return float(self.variable_b2.get())

    def getC1Value(self):
        return float(self.variable_c1.get())

    def getC2Value(self):
        return float(self.variable_c2.get())

    def getDxValue(self):
        return float(self.variable_dx.get())

    def getDyValue(self):
        return float(self.variable_dy.get())

    def getDzValue(self):
        return float(self.variable_dz.get())

    def getRxValue(self):
        return float(self.variable_rx.get())

    def getRyValue(self):
        return float(self.variable_ry.get())

    def getRzValue(self):
        return float(self.variable_rz.get())

    def getRadiusValue(self):
        return float(self.variable_radius.get())

    def getH1Value(self):
        return float(self.variable_h1.get())

    def getH2Value(self):
        return float(self.variable_h2.get())

    def getCylDxValue(self):
        return float(self.variable_cyl_dx.get())

    def getCylDyValue(self):
        return float(self.variable_cyl_dy.get())

    def getCylDzValue(self):
        return float(self.variable_cyl_dz.get())

    def getCylRxValue(self):
        return float(self.variable_cyl_rx.get())

    def getCylRyValue(self):
        return float(self.variable_cyl_ry.get())

    def getCylRzValue(self):
        return float(self.variable_cyl_rz.get())



    def setValueA1(self, value):
        self.variable_a1.set(value)

    def setValueA2(self, value):
        self.variable_a2.set(value)

    def setValueB1(self, value):
        self.variable_b1.set(value)

    def setValueB2(self, value):
        self.variable_b2.set(value)

    def setValueC1(self, value):
        self.variable_c1.set(value)

    def setValueC2(self, value):
        self.variable_c2.set(value)

    def setDxValue(self, value):
        self.variable_dx.set(value)

    def setDyValue(self, value):
        self.variable_dy.set(value)

    def setDzValue(self, value):
        self.variable_dz.set(value)

    def setRxValue(self, value):
        self.variable_rx.set(value)

    def setRyValue(self, value):
        self.variable_ry.set(value)

    def setRzValue(self, value):
        self.variable_rz.set(value)



    def setRadiusValue(self, value):
        self.variable_radius.set(value)

    def setH1Value(self, value):
        self.variable_h1.set(value)

    def setH2Value(self, value):
        self.variable_h2.set(value)

    def setCylDxValue(self, value):
        self.variable_cyl_dx.set(value)

    def setCylDyValue(self, value):
        self.variable_cyl_dy.set(value)

    def setCylDzValue(self, value):
        self.variable_cyl_dz.set(value)

    def setCylRxValue(self, value):
        self.variable_cyl_rx.set(value)

    def setCylRyValue(self, value):
        self.variable_cyl_ry.set(value)

    def setCylRzValue(self, value):
        self.variable_cyl_rz.set(value)

class UserInteraction:
    def __init__(self):
        # Planar Elements
        self.current_element_number_planar = 0
        self.number_elements_planar = 0

        self.element_centers = []
        self.element_directions_1 = []
        self.element_directions_2 = []
        self.element_directions_3 = []
        self.element_a1 = []
        self.element_a2 = []
        self.element_b1 = []
        self.element_b2 = []
        self.element_c1 = []
        self.element_c2 = []

        # Cylinders
        self.current_element_number_cylinders = 0
        self.number_elements_cylinders = 0

        self.element_radius = []
        self.element_p1 = []
        self.element_p2 = []
        self.element_axis = []

        self.model_mesh = 0
        self.render = True
        self.accept = False

    def getElementProposalsFromMessage(self, msg):
        self.number_elements_planar = len(msg.centers)
        self.number_elements_cylinders = len(msg.radius)

        for i in range(0, self.number_elements_planar):
            paras = msg.magnitudes[i]

            cur_a1 = paras.params[0]
            cur_a2 = paras.params[1]
            cur_b1 = paras.params[2]
            cur_b2 = paras.params[3]
            cur_c1 = paras.params[4]
            cur_c2 = paras.params[5]

            self.element_a1.append(cur_a1)
            self.element_a2.append(cur_a2)
            self.element_b1.append(cur_b1)
            self.element_b2.append(cur_b2)
            self.element_c1.append(cur_c1)
            self.element_c2.append(cur_c2)

            cur_dir_1 = np.array([])
            cur_dir_2 = np.array([])
            cur_dir_3 = np.array([])
            cur_dir_1 = np.append(cur_dir_1, msg.dir_1[i].x)
            cur_dir_1 = np.append(cur_dir_1, msg.dir_1[i].y)
            cur_dir_1 = np.append(cur_dir_1, msg.dir_1[i].z)
            cur_dir_2 = np.append(cur_dir_2, msg.dir_2[i].x)
            cur_dir_2 = np.append(cur_dir_2, msg.dir_2[i].y)
            cur_dir_2 = np.append(cur_dir_2, msg.dir_2[i].z)
            cur_dir_3 = np.append(cur_dir_3, msg.dir_3[i].x)
            cur_dir_3 = np.append(cur_dir_3, msg.dir_3[i].y)
            cur_dir_3 = np.append(cur_dir_3, msg.dir_3[i].z)
            self.element_directions_1.append(cur_dir_1)
            self.element_directions_2.append(cur_dir_2)
            self.element_directions_3.append(cur_dir_3)

            cur_center = np.array([])
            cur_center = np.append(cur_center, msg.centers[i].x)
            cur_center = np.append(cur_center, msg.centers[i].y)
            cur_center = np.append(cur_center, msg.centers[i].z)
            self.element_centers.append(cur_center)

        for i in range(0, self.number_elements_cylinders):
            cur_radius = msg.radius[i]
            p1 = msg.cyl_p1[i]
            p2 = msg.cyl_p2[i]

            norm = np.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2 + (p1.z - p2.z) ** 2)
            axis_x = (p2.x - p1.x) / norm
            axis_y = (p2.y - p1.y) / norm
            axis_z = (p2.z - p1.z) / norm
            axis = np.array([axis_x, axis_y, axis_z])

            self.element_radius.append(cur_radius)
            self.element_p1.append(np.array([p1.x, p1.y, p1.z]))
            self.element_p2.append(np.array([p2.x, p2.y, p2.z]))
            self.element_axis.append(axis)

    def loadCommandInterface(self):

        done_meshes = []

        while (self.current_element_number_planar < self.number_elements_planar):
            idx = self.current_element_number_planar
            rospy.loginfo(str(self.current_element_number_planar))

            a1 = self.element_a1[idx]
            a2 = self.element_a2[idx]
            b1 = self.element_b1[idx]
            b2 = self.element_b2[idx]
            c1 = self.element_c1[idx]
            c2 = self.element_c2[idx]

            center = self.element_centers[idx]

            dir_1 = self.element_directions_1[idx]
            dir_2 = self.element_directions_2[idx]
            dir_3 = self.element_directions_3[idx]

            global manipulationPanel
            manipulationPanel.setValueA1(a1)
            manipulationPanel.setValueA2(a2)
            manipulationPanel.setValueB1(b1)
            manipulationPanel.setValueB2(b2)
            manipulationPanel.setValueC1(c1)
            manipulationPanel.setValueC2(c2)
            manipulationPanel.setDxValue(0.0)
            manipulationPanel.setDyValue(0.0)
            manipulationPanel.setDzValue(0.0)
            manipulationPanel.setRxValue(0.0)
            manipulationPanel.setRyValue(0.0)
            manipulationPanel.setRzValue(0.0)

            p1, p2, p3, p4, p5, p6, p7, p8 = self.getPointsFromParameters(center, dir_1, dir_2, dir_3, a1,
                                                                          a2, b1, b2, c1, c2)
            P, T = self.prepareDatapointsForMeshing(p1, p2, p3, p4, p5, p6, p7, p8)

            points = o3d.utility.Vector3dVector(P)
            indices = o3d.utility.Vector3iVector(T)
            element = o3d.geometry.TriangleMesh(points, indices)
            element.paint_uniform_color(np.array([1, 0, 0]))

            o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)
            vis = o3d.visualization.VisualizerWithKeyCallback()
            vis.register_key_callback(257, self.callbackAccept)  # enter
            vis.register_key_callback(259, self.callbackReject)  # backspace

            vis.create_window()
            vis.add_geometry(self.model_mesh)
            vis.add_geometry(element)
            for el in done_meshes:
                vis.add_geometry(el)

            vis.get_render_option().light_on = True
            vis.get_render_option().mesh_show_wireframe = True
            vis.get_render_option().mesh_show_back_face = True

            manipulationPanel.activatePlanarElements()
            manipulationPanel.deactivateCylinders()

            model_vertices = np.asarray(self.model_mesh.vertices)
            num_model_vertices = model_vertices.shape[0]

            model_cloud = o3d.geometry.PointCloud(self.model_mesh.vertices)
            model_kd_tree = o3d.geometry.KDTreeFlann(model_cloud)

            color_map = np.zeros(shape = (num_model_vertices, 3))

            self.render = True
            self.accept = False
            while (self.render):
                a1_user = manipulationPanel.getA1Value()
                a2_user = manipulationPanel.getA2Value()
                b1_user = manipulationPanel.getB1Value()
                b2_user = manipulationPanel.getB2Value()
                c1_user = manipulationPanel.getC1Value()
                c2_user = manipulationPanel.getC2Value()
                dx_user = manipulationPanel.getDxValue()
                dy_user = manipulationPanel.getDyValue()
                dz_user = manipulationPanel.getDzValue()
                rx_user = manipulationPanel.getRxValue()
                ry_user = manipulationPanel.getRyValue()
                rz_user = manipulationPanel.getRzValue()

                user_translation = np.array([dx_user, dy_user, dz_user])
                user_rotation = element.get_rotation_matrix_from_xyz(np.array([rx_user * np.pi / 180.,
                                                                     ry_user * np.pi / 180.,
                                                                     rz_user * np.pi / 180.]))

                p1_new, p2_new, p3_new, p4_new, \
                p5_new, p6_new, p7_new, p8_new = self.getPointsFromParameters(center, dir_1, dir_2, dir_3,
                                                                              a1_user, a2_user,
                                                                              b1_user, b2_user,
                                                                              c1_user, c2_user)

                p_updated, t_updated = self.prepareDatapointsForMeshing(p1_new, p2_new, p3_new, p4_new, p5_new, p6_new,
                                                                        p7_new, p8_new)
                points_updated = o3d.utility.Vector3dVector(p_updated)
                element.vertices = points_updated
                element.translate(user_translation)
                element.rotate(user_rotation)
                p_updated = np.asarray(element.vertices)

                #Update colors
                num_element_vertices = p_updated.shape[0]

                color_map[:, 0] = 0
                color_map[:, 1] = 0
                color_map[:, 2] = 1
                for i in range(0, num_element_vertices):
                    [k, idx, _] = model_kd_tree.search_radius_vector_3d(np.transpose(p_updated[i, :]), 0.02)
                    for j in range(0, k):
                        idx_to_change = idx[j]
                        color_map[idx_to_change, 0] = 1
                        color_map[idx_to_change, 1] = 0.67
                        color_map[idx_to_change, 2] = 0

                self.model_mesh.vertex_colors = o3d.utility.Vector3dVector(color_map)

                vis.update_geometry(element)
                vis.update_geometry(self.model_mesh)
                vis.poll_events()
                vis.update_renderer()
                if (self.render == False):
                    break

            if (self.accept):
                element.paint_uniform_color(np.array([0, 1, 0]))
                done_meshes.append(element)

            vis.destroy_window()
            self.current_element_number_planar = self.current_element_number_planar + 1

        manipulationPanel.deactivatePlanarElements()
        manipulationPanel.activateCylinders()


        while (self.current_element_number_cylinders < self.number_elements_cylinders):
            idx = self.current_element_number_cylinders
            rospy.loginfo(str(self.current_element_number_cylinders))

            radius = self.element_radius[idx]
            p1 = self.element_p1[idx]
            p2 = self.element_p2[idx]
            axis = self.element_axis[idx]

            height = np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2 + (p1[2] - p2[2]) ** 2)
            # global manipulationPanel

            info_1 = "height is: " + str(height)
            info_2 = "radius is: " + str(radius)
            rospy.loginfo(info_1)
            rospy.loginfo(info_2)
            cyl_mesh = o3d.geometry.TriangleMesh()
            cyl_mesh = cyl_mesh.create_cylinder(radius, height, 20, 10)
            cyl_mesh.paint_uniform_color(np.array([1, 0, 0]))

            vec1 = np.array([0, 0, 1])
            vec2 = axis
            # Compute transformation matrix
            # Source: https://stackoverflow.com/questions/45142959/calculate-rotation-matrix-to-align-two-vectors-in-3d-space
            a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
            v = np.cross(a, b)
            rotation_matrix = np.eye(3)
            if any(v):
                c = np.dot(a, b)
                s = np.linalg.norm(v)
                kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
                rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))

            transformation_matrix = np.eye(4)
            transformation_matrix[0:3, 0:3] = rotation_matrix

            cyl_mesh.transform(transformation_matrix)

            center_2 = np.array([(p1[0] + p2[0]) / 2.0, (p1[1] + p2[1]) / 2.0, (p1[2] + p2[2]) / 2.0])
            cyl_mesh.translate(center_2, False)

            manipulationPanel.setRadiusValue(radius)
            manipulationPanel.setH1Value(height / 2.0)
            manipulationPanel.setH2Value(height / 2.0)
            manipulationPanel.setCylDxValue(0.0)
            manipulationPanel.setCylDyValue(0.0)
            manipulationPanel.setCylDzValue(0.0)
            manipulationPanel.setCylRxValue(0.0)
            manipulationPanel.setCylRyValue(0.0)
            manipulationPanel.setCylRzValue(0.0)


            o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)
            vis = o3d.visualization.VisualizerWithKeyCallback()
            vis.register_key_callback(257, self.callbackAccept)  # enter
            vis.register_key_callback(259, self.callbackReject)  # backspace

            vis.create_window()
            vis.add_geometry(self.model_mesh)
            vis.add_geometry(cyl_mesh)
            for el in done_meshes:
                vis.add_geometry(el)

            vis.get_render_option().light_on = True
            vis.get_render_option().mesh_show_wireframe = True
            vis.get_render_option().mesh_show_back_face = True

            self.render = True
            self.accept = False
            while (self.render):
                radius_user = manipulationPanel.getRadiusValue()
                h1_user = manipulationPanel.getH1Value()
                h2_user = manipulationPanel.getH2Value()
                dx_user = manipulationPanel.getCylDxValue()
                dy_user = manipulationPanel.getCylDyValue()
                dz_user = manipulationPanel.getCylDzValue()
                rx_user = manipulationPanel.getCylRxValue()
                ry_user = manipulationPanel.getCylRyValue()
                rz_user = manipulationPanel.getCylRzValue()

                user_translation = np.array([dx_user, dy_user, dz_user])
                user_rotation = element.get_rotation_matrix_from_xyz(np.array([rx_user * np.pi / 180.,
                                                                               ry_user * np.pi / 180.,
                                                                               rz_user * np.pi / 180.]))

                height_user = h1_user + h2_user
                #Update center
                p1_user = center_2 + h1_user * axis
                p2_user = center_2 - h2_user * axis
                center_user = np.array([(p1_user[0] + p2_user[0]) / 2.0, (p1_user[1] + p2_user[1]) / 2.0, (p1_user[2] + p2_user[2]) / 2.0])

                cyl_user = o3d.geometry.TriangleMesh()
                cyl_user = cyl_user.create_cylinder(radius_user, height_user, 20, 10)
                cyl_user.paint_uniform_color(np.array([1, 0, 0]))
                cyl_user.transform(transformation_matrix)
                cyl_user.translate(center_user)

                cyl_user.translate(user_translation)
                cyl_user.rotate(user_rotation)

                cyl_mesh.vertices = cyl_user.vertices
                cyl_mesh.triangles = cyl_user.triangles

                vis.update_geometry(cyl_mesh)
                vis.poll_events()
                vis.update_renderer()
                if (self.render == False):
                    break

            if (self.accept):
                cyl_mesh.paint_uniform_color(np.array([0, 1, 0]))
                done_meshes.append(cyl_mesh)

            vis.destroy_window()
            self.current_element_number_cylinders = self.current_element_number_cylinders + 1

        final_mesh = o3d.io.read_triangle_mesh(BUILDING_MODEL_PATH_)
        for el in done_meshes:
            final_mesh = final_mesh + el

        added_mesh = o3d.geometry.TriangleMesh()
        for el in done_meshes:
            added_mesh = added_mesh + el

        final_mesh.merge_close_vertices(0.02)
        final_mesh.remove_duplicated_vertices()
        o3d.io.write_triangle_mesh("/home/philipp/Schreibtisch/final_reconstructed_mesh.ply", final_mesh)
        o3d.io.write_triangle_mesh("/home/philipp/Schreibtisch/added_reconstructed_mesh.ply", added_mesh)

    def setModelDataForO3D(self):
        mesh = o3d.io.read_triangle_mesh(BUILDING_MODEL_PATH_)
        mesh.paint_uniform_color(np.array([0, 0, 1]))
        self.model_mesh = mesh

    def getPointsFromParameters(self, center, dir1, dir2, dir3, a1, a2, b1, b2, c1, c2):
        p1 = np.array([])
        p2 = np.array([])
        p3 = np.array([])
        p4 = np.array([])
        p5 = np.array([])
        p6 = np.array([])
        p7 = np.array([])
        p8 = np.array([])
        for i in range(0, 3):
            p1_i = center[i] + dir1[i] * a1 + dir2[i] * b1 + dir3[i] * c1
            p2_i = center[i] + dir1[i] * a1 + dir2[i] * b1 + dir3[i] * c2
            p3_i = center[i] + dir1[i] * a1 + dir2[i] * b2 + dir3[i] * c1
            p4_i = center[i] + dir1[i] * a1 + dir2[i] * b2 + dir3[i] * c2

            p5_i = center[i] + dir1[i] * a2 + dir2[i] * b1 + dir3[i] * c1
            p6_i = center[i] + dir1[i] * a2 + dir2[i] * b1 + dir3[i] * c2
            p7_i = center[i] + dir1[i] * a2 + dir2[i] * b2 + dir3[i] * c1
            p8_i = center[i] + dir1[i] * a2 + dir2[i] * b2 + dir3[i] * c2

            p1 = np.append(p1, p1_i)
            p2 = np.append(p2, p2_i)
            p3 = np.append(p3, p3_i)
            p4 = np.append(p4, p4_i)
            p5 = np.append(p5, p5_i)
            p6 = np.append(p6, p6_i)
            p7 = np.append(p7, p7_i)
            p8 = np.append(p8, p8_i)
        return p1, p2, p3, p4, p5, p6, p7, p8

    def prepareDatapointsForMeshing(self, p1, p2, p3, p4, p5, p6, p7, p8):
        P = np.array([[p1[0], p1[1], p1[2]],
                      [p2[0], p2[1], p2[2]],
                      [p3[0], p3[1], p3[2]],
                      [p4[0], p4[1], p4[2]],
                      [p5[0], p5[1], p5[2]],
                      [p6[0], p6[1], p6[2]],
                      [p7[0], p7[1], p7[2]],
                      [p8[0], p8[1], p8[2]]])

        T = [[0, 4, 2], [4, 6, 2], [4, 1, 0], [5, 1, 4], [6, 5, 4], [7, 5, 6], [1, 3, 5], [7, 5, 3], [3, 2, 7],
             [6, 7, 2], [1, 0, 2], [3, 1, 2]]

        return P, T

    def callbackAccept(self, vis):
        rospy.loginfo("Pressed n ")
        self.render = False
        self.accept = True

    def callbackReject(self, vis):
        rospy.loginfo("Pressed r ")
        self.render = False
        self.accept = False

def callback(msg):
    rospy.loginfo(rospy.get_caller_id() + "Nr planar elements %d", len(msg.centers))
    rospy.loginfo(rospy.get_caller_id() + "Nr cyl elements %d", len(msg.radius))
    graphical_interface = UserInteraction()
    graphical_interface.setModelDataForO3D()
    graphical_interface.getElementProposalsFromMessage(msg)
    graphical_interface.loadCommandInterface()


def model_integration():
    rospy.init_node('model_integration_node', anonymous=True)

    global BUILDING_MODEL_PATH_
    BUILDING_MODEL_PATH_ = rospy.get_param("BuildingModelMeshFile")

    rospy.loginfo("Initialized Python_node")
    rospy.Subscriber("element_proposals", element_proposals, callback)

    global manipulationPanel
    manipulationPanel = ManipulationPanel()
    manipulationPanel.startMainloop()
    # rospy.spin()


if __name__ == '__main__':
    model_integration()
