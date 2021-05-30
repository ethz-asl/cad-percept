#!/usr/bin/env python

import open3d as o3d

import rospy
from cpt_reconstruction.msg import element_proposals, parameters
from geometry_msgs.msg import Vector3

import numpy as np

from plyfile import PlyData, PlyElement

import Tkinter as tk
import threading

manipulationPanel = None


class ManipulationPanel:
    def __init__(self):
        self.root = tk.Tk()
        self.root.protocol("WM_DELETE_WINDOW", self.callback)

        self.variable_a1 = tk.DoubleVar(value=0)
        self.spinbox_a1 = tk.Spinbox(self.root, from_=0, to=100, increment=0.05, width=5, justify=tk.RIGHT,
                                     textvariable=self.variable_a1)
        self.spinbox_a1.pack()

        self.variable_a2 = tk.DoubleVar(value=0)
        self.spinbox_a2 = tk.Spinbox(self.root, from_=0, to=100, increment=0.05, width=5, justify=tk.RIGHT,
                                     textvariable=self.variable_a2)
        self.spinbox_a2.pack()

        self.variable_b1 = tk.DoubleVar(value=0)
        self.spinbox_b1 = tk.Spinbox(self.root, from_=0, to=100, increment=0.05, width=5, justify=tk.RIGHT,
                                     textvariable=self.variable_b1)
        self.spinbox_b1.pack()

        self.variable_b2 = tk.DoubleVar(value=0)
        self.spinbox_b2 = tk.Spinbox(self.root, from_=0, to=100, increment=0.05, width=5, justify=tk.RIGHT,
                                     textvariable=self.variable_b2)
        self.spinbox_b2.pack()

        self.variable_c1 = tk.DoubleVar(value=0)
        self.spinbox_c1 = tk.Spinbox(self.root, from_=0, to=100, increment=0.05, width=5, justify=tk.RIGHT,
                                     textvariable=self.variable_c1)
        self.spinbox_c1.pack()

        self.variable_c2 = tk.DoubleVar(value=0)
        self.spinbox_c2 = tk.Spinbox(self.root, from_=0, to=100, increment=0.05, width=5, justify=tk.RIGHT,
                                     textvariable=self.variable_c2)
        self.spinbox_c2.pack()

    def startMainloop(self):
        self.root.mainloop()

    def callback(self):
        self.root.quit()

    def destroy(self):
        self.root.destroy()

    def quit(self):
        self.root.quit()

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


class UserInteraction:
    def __init__(self):
        self.current_element_number = 0
        self.number_elements = 0
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

        self.model_mesh = 0
        self.render = True

        self.ptr_a1 = 0
        self.ptr_a2 = 0
        self.ptr_b1 = 0
        self.ptr_b2 = 0
        self.ptr_c1 = 0
        self.ptr_c2 = 0

    def getElementProposalsFromMessage(self, msg):
        self.number_elements = len(msg.ids)

        for i in range(0, self.number_elements):
            paras = msg.magnitudes[i]

            cur_a1 = np.array([])
            cur_a2 = np.array([])
            cur_b1 = np.array([])
            cur_b2 = np.array([])
            cur_c1 = np.array([])
            cur_c2 = np.array([])
            for j in range(0, len(paras.a1)):
                cur_a1 = np.append(cur_a1, paras.a1[j])
            for j in range(0, len(paras.a2)):
                cur_a2 = np.append(cur_a2, paras.a2[j])
            for j in range(0, len(paras.b1)):
                cur_b1 = np.append(cur_b1, paras.b1[j])
            for j in range(0, len(paras.b2)):
                cur_b2 = np.append(cur_b2, paras.b2[j])
            for j in range(0, len(paras.c1)):
                cur_c1 = np.append(cur_c1, paras.c1[j])
            for j in range(0, len(paras.c2)):
                cur_c2 = np.append(cur_c2, paras.c2[j])

            cur_a1 = np.sort(cur_a1)
            cur_a2 = -np.sort(-cur_a2)
            cur_b1 = np.sort(cur_b1)
            cur_b2 = -np.sort(-cur_b2)
            cur_c1 = np.sort(cur_c1)
            cur_c2 = -np.sort(-cur_c2)

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

    def loadCommandInterface(self):

        done_meshes = []

        while (self.current_element_number < self.number_elements):
            idx = self.current_element_number
            rospy.loginfo(str(self.current_element_number))

            a1 = self.element_a1[idx]
            a2 = self.element_a2[idx]
            b1 = self.element_b1[idx]
            b2 = self.element_b2[idx]
            c1 = self.element_c1[idx]
            c2 = self.element_c2[idx]

            self.ptr_a1 = len(a1) - 1
            self.ptr_a2 = len(a2) - 1
            self.ptr_b1 = len(b1) - 1
            self.ptr_b2 = len(b2) - 1
            self.ptr_c1 = len(c1) - 1
            self.ptr_c2 = len(c2) - 1

            center = self.element_centers[idx]

            dir_1 = self.element_directions_1[idx]
            dir_2 = self.element_directions_2[idx]
            dir_3 = self.element_directions_3[idx]

            global manipulationPanel
            manipulationPanel.setValueA1(a1[self.ptr_a1])
            manipulationPanel.setValueA2(- a2[self.ptr_a2])
            manipulationPanel.setValueB1(b1[self.ptr_b1])
            manipulationPanel.setValueB2(- b2[self.ptr_b2])
            manipulationPanel.setValueC1(c1[self.ptr_c1])
            manipulationPanel.setValueC2(- c2[self.ptr_c2])

            p1, p2, p3, p4, p5, p6, p7, p8 = self.getPointsFromParameters(center, dir_1, dir_2, dir_3, a1[self.ptr_a1],
                                                                          a2[self.ptr_a2], b1[self.ptr_b1],
                                                                          b2[self.ptr_b2], c1[self.ptr_c1],
                                                                          c2[self.ptr_c2])
            P, T = self.prepareDatapointsForMeshing(p1, p2, p3, p4, p5, p6, p7, p8)

            points = o3d.utility.Vector3dVector(P)
            indices = o3d.utility.Vector3iVector(T)
            element = o3d.geometry.TriangleMesh(points, indices)
            element.paint_uniform_color(np.array([1, 0, 0]))

            o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)
            vis = o3d.visualization.VisualizerWithKeyCallback()
            vis.register_key_callback(65, self.callbackExit)  # a

            vis.create_window()
            vis.add_geometry(self.model_mesh)
            vis.add_geometry(element)
            for el in done_meshes:
                vis.add_geometry(el)

            vis.get_render_option().light_on = True
            vis.get_render_option().mesh_show_wireframe = True
            vis.get_render_option().mesh_show_back_face = True

            self.render = True
            while (self.render):
                a1_user = manipulationPanel.getA1Value()
                a2_user = -manipulationPanel.getA2Value()
                b1_user = manipulationPanel.getB1Value()
                b2_user = -manipulationPanel.getB2Value()
                c1_user = manipulationPanel.getC1Value()
                c2_user = -manipulationPanel.getC2Value()

                p1_new, p2_new, p3_new, p4_new, \
                p5_new, p6_new, p7_new, p8_new = self.getPointsFromParameters(center, dir_1, dir_2, dir_3,
                                                                              a1_user, a2_user,
                                                                              b1_user, b2_user,
                                                                              c1_user, c2_user)

                P_updated, T_updated = self.prepareDatapointsForMeshing(p1_new, p2_new, p3_new, p4_new, p5_new, p6_new,
                                                                        p7_new, p8_new)
                points_updated = o3d.utility.Vector3dVector(P_updated)
                element.vertices = points_updated
                vis.update_geometry(element)
                vis.poll_events()
                vis.update_renderer()
                if (self.render == False):
                    break

            element.paint_uniform_color(np.array([0, 1, 0]))
            done_meshes.append(element)

            vis.destroy_window()
            self.current_element_number = self.current_element_number + 1

    def setModelDataForO3D(self):
        mesh = o3d.io.read_triangle_mesh('/home/philipp/Schreibtisch/data/CLA_MissingParts_3.ply')
        mesh.paint_uniform_color(np.array([0, 0, 1]))
        self.model_mesh = mesh

    def prepeareModelData(self):
        plydata = PlyData.read('/home/philipp/Schreibtisch/data/CLA_MissingParts_3.ply')

        num_faces = plydata['face'].count
        x = np.array([])
        y = np.array([])
        z = np.array([])
        t = []
        for i in range(0, num_faces):
            vertices = plydata['face'][i][0]

            v1 = vertices[0]
            v2 = vertices[1]
            v3 = vertices[2]
            t.append([v1, v2, v3])
            if (len(vertices) > 3):
                print(len(vertices))

        num_vertices = plydata['vertex'].count
        for i in range(0, num_vertices):
            p = plydata['vertex'][i]
            x = np.append(x, p[0])
            y = np.append(y, p[1])
            z = np.append(z, p[2])
        return x, y, z, t

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

    def callbackExit(self, vis):
        rospy.loginfo("Pressed a ")
        self.render = False


def callback(msg):
    rospy.loginfo(rospy.get_caller_id() + "I heard %d", len(msg.ids))
    graphical_interface = UserInteraction()
    graphical_interface.setModelDataForO3D()
    graphical_interface.getElementProposalsFromMessage(msg)
    graphical_interface.loadCommandInterface()


def model_integration():
    rospy.init_node('model_integration_node', anonymous=True)
    rospy.loginfo("Initialized Python_node")
    rospy.Subscriber("element_proposals", element_proposals, callback)

    global manipulationPanel
    manipulationPanel = ManipulationPanel()
    manipulationPanel.startMainloop()
    # rospy.spin()


if __name__ == '__main__':
    model_integration()
