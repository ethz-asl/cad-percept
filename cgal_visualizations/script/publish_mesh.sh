rostopic pub /test_mesh cgal_msgs/ProbabilisticMesh "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: 'world'
mesh:
  triangles:
  - vertex_indices: [0, 1, 2]
  vertices:
  - {x: 0.0, y: 0.0, z: 0.0}
  - {x: 0.0, y: 1.0, z: 1.0}
  - {x: 1.0, y: 1.0, z: 0.0}
normals:
- {x: 0.0, y: 0.0, z: 0.0}
cov_vertices:
- {x: 0.0, y: 0.0, z: 0.0}
- {x: 0.0, y: 0.0, z: 0.0}
- {x: 0.0, y: 0.0, z: 0.0}"
