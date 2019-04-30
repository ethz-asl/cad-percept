#include <cgal_conversions/mesh_conversions.h>
#include <cgal_definitions/cgal_typedefs.h>
#include <gflags/gflags.h>
#include <gtest/gtest.h>
#include <cgal>

#include <cgal_msgs/TriangleMesh.h>

using namespace cad_percept::cgal;

TEST(CGALConversionsTest, triangle_mesh_to_msg) {
  SurfaceMesh m;
  // create 4 vertices
  vertex_descriptor a = m.add_vertex(Point(1, 0, 0));
  vertex_descriptor b = m.add_vertex(Point(0, 1, 0));
  vertex_descriptor c = m.add_vertex(Point(0, 0, 0));
  vertex_descriptor d = m.add_vertex(Point(1, 1, 0));
  // create 3 triangles
  m.add_face(a, b, c);
  m.add_face(b, c, d);
  m.add_face(a, b, d);
  // get message from mesh
  TriangleMeshMsg msg = triangleMeshToMsg(m);
  ExpectElementsAreArray(msg.vertices,
                         {{1, 0, 0}, {0, 1, 0}, {0, 0, 0}, {1, 1, 0}});
}

TEST(CGALConversionsTest, msg_to_triangle_mesh) {
  SurfaceMesh m;
  // create 4 vertices
  vertex_descriptor a = m.add_vertex(Point(1, 0, 0));
  vertex_descriptor b = m.add_vertex(Point(0, 1, 0));
  vertex_descriptor c = m.add_vertex(Point(0, 0, 0));
  vertex_descriptor d = m.add_vertex(Point(1, 1, 0));
  // create 3 triangles
  m.add_face(a, b, c);
  m.add_face(b, c, d);
  m.add_face(a, b, d);
  // get message from mesh
  TriangleMeshMsg msg = triangleMeshToMsg(m);
  
}
