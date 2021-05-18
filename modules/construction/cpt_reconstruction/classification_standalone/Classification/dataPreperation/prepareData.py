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
                    for j in range (0, len(x_data)):
                            new_data.append((x_data[j], y_data[j], z_data[j], class_number))
            if (len(new_data)> 50000):
                break
        if (len(new_data)> 50000):
            break
    write_data = np.array(new_data, dtype=[('x', 'f8'), ('y', 'f8'), ('z', 'f8'), ('label', 'i4')])
    el = PlyElement.describe(write_data, 'vertex')
    PlyData([el]).write('training_data_area1_temp.ply')


def main_prediction():
    data = np.loadtxt("/home/philipp/Schreibtisch/outliers_ros.txt", max_rows= 50000)

    x_data = data[:, 0]
    y_data = data[:, 1]
    z_data = data[:, 2]
    new_data = []
    for j in range(0, len(x_data)):
        new_data.append((x_data[j], y_data[j], z_data[j]))

    write_data = np.array(new_data, dtype=[('x', 'f8'), ('y', 'f8'), ('z', 'f8')])
    el = PlyElement.describe(write_data, 'vertex')
    PlyData([el]).write('prediction_data.ply')


if __name__ == "__main__":
    main_prediction()