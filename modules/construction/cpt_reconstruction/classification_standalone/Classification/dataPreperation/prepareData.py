from plyfile import PlyData, PlyElement
import numpy as np
import random
import os
import open3d as o3d
import trimesh

def main_training():

    rootdir = "/home/philipp/reconstruction_ws/src/cad-percept/modules/construction/cpt_reconstruction/classification_standalone/Classification/dataPreperation/Area_1"

    #lables = ["beam", "board", "bookcase", "ceiling", "chair", "clutter", "column", "door", "floor", "sofa", "table", "wall", "window"]

    lables = ["wall", "ceiling", "beam", "column","floor", "clutter", "board", "bookcase",  "chair",   "door",  "sofa", "table", "window"]

    new_data = []
    for subdir, dirs, files in os.walk(rootdir):
        for file in files:
            print(file)
            for i in range(0, len(lables)):
                if lables[i] in file:
                    class_number = i
                    if (i >= 5):
                        class_number = 5
                    print(file, " is class ", i, " -> ", lables[class_number])
                    data = np.loadtxt(os.path.join(subdir, file))
                    x_data = data[:, 0]
                    y_data = data[:, 1]
                    z_data = data[:, 2]
                    r_noise = random.random()
                    r_size = random.random()
                    step = 1
                    if (r_size < 0.2):
                        step = 3
                    elif (r_size < 0.4):
                        step = 6
                    elif (r_size < 0.6):
                        step = 9
                    elif (r_size < 0.8):
                        step = 12

                    if ( r_noise < 0.25):
                        for j in range (0, len(x_data), step):
                                new_data.append((x_data[j], y_data[j], z_data[j], class_number))
                    elif (r_noise < 0.5):
                        for j in range (0, len(x_data), step):
                                r1 = random.gauss(0, 0.01)
                                r2 = random.gauss(0, 0.01)
                                r3 = random.gauss(0, 0.01)
                                new_data.append((x_data[j] + r1, y_data[j] + r2, z_data[j] + r3, class_number))
                    elif(r_noise < 0.75):
                        for j in range (0, len(x_data), step):
                                r1 = random.gauss(0, 0.02)
                                r2 = random.gauss(0, 0.02)
                                r3 = random.gauss(0, 0.02)
                                new_data.append((x_data[j] + r1, y_data[j] + r2, z_data[j] + r3, class_number))
                    else:
                        for j in range (0, len(x_data), step):
                                r1 = random.gauss(0, 0.03)
                                r2 = random.gauss(0, 0.03)
                                r3 = random.gauss(0, 0.03)
                                new_data.append((x_data[j] + r1, y_data[j] + r2, z_data[j] + r3, class_number))


    write_data = np.array(new_data, dtype=[('x', 'f8'), ('y', 'f8'), ('z', 'f8'), ('label', 'i4')])
    el = PlyElement.describe(write_data, 'vertex')
    PlyData([el]).write('training_data_area1_random.ply')

    write_data = np.array(new_data, dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('label', 'i4')])
    el = PlyElement.describe(write_data, 'vertex')
    PlyData([el]).write('training_data_area1_random_float.ply')

def main_prediction():
    data = np.loadtxt("/home/philipp/Schreibtisch/outliers_ros_500k.txt")

    scale = 1.0
    if (True):
        scale = 1.0/1000.0

    x_data = data[:, 0]
    y_data = data[:, 1]
    z_data = data[:, 2]
    new_data = []
    for j in range(0, len(x_data), 6):
        new_data.append((x_data[j] * scale, y_data[j] * scale, z_data[j] * scale))

    write_data = np.array(new_data, dtype=[('x', 'f8'), ('y', 'f8'), ('z', 'f8')])
    el = PlyElement.describe(write_data, 'vertex')
    PlyData([el]).write('prediction_data_cla_outliers_500k.ply')


if __name__ == "__main__":
    main_training()