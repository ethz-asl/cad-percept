rostopic pub /test_mesh cgal_msgs/ColoredMesh "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: 'map'
mesh:
  triangles:
  - vertex_indices: [0, 1, 2]
  - vertex_indices: [2, 1, 3]
  vertices:
  - {x: 0.0, y: 0.0, z: 0.0}
  - {x: 0.0, y: 1.0, z: 1.0}
  - {x: 1.0, y: 1.0, z: 0.0}
  - {x: 3.0, y: 2.0, z: 0.0}
color: {r: 1.0, g: 0.0, b: 0.0, a: 0.8}
colors:
  - {r: 0.0, g: 1.0, b: 0.0, a: 0.8}
  - {r: 1.0, g: 0.0, b: 0.0, a: 0.8}"