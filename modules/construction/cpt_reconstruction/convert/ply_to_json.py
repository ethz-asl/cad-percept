from plyfile import PlyData, PlyElement

import compas
from compas.datastructures import Mesh

## Modifiy Paths
ply_path = "example_mesh.ply"
json_path = "example_mesh.json"

# Temp File storing ply with ascii encoding
temp_path = "example_mesh_ascii.ply"

def main():
    plydata = PlyData.read(ply_path)
    PlyData(plydata, text=True).write(temp_path)
    
    mesh = Mesh.from_ply(temp_path)
    mesh.to_json(json_path, True)

if __name__ == '__main__':
    main()