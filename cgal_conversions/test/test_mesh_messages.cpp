#include <gflags/gflags.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cgal_conversions/mesh_conversions.h>
#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_msgs/TriangleMesh.h>

using namespace cad_percept::cgal;

// A modifier creating a triangle with the incremental builder.
template <class HDS>
class TestingMesh : public CGAL::Modifier_base<HDS> {
 public:
  TestingMesh() {}
  void operator()(HDS& hds) {
    // Postcondition: hds is a valid polyhedral surface.
    CGAL::Polyhedron_incremental_builder_3<HDS> B(hds, true);
    B.begin_surface(4, 3);
    B.add_vertex(Point(0, 0, 0));
    B.add_vertex(Point(1, 0, 0));
    B.add_vertex(Point(0, 1, 0));
    B.add_vertex(Point(1, 1, 0));

    B.begin_facet();
    B.add_vertex_to_facet(0);
    B.add_vertex_to_facet(1);
    B.add_vertex_to_facet(2);
    B.end_facet();

    B.begin_facet();
    B.add_vertex_to_facet(1);
    B.add_vertex_to_facet(3);
    B.add_vertex_to_facet(2);
    B.end_facet();

    B.begin_facet();
    B.add_vertex_to_facet(0);
    B.add_vertex_to_facet(3);
    B.add_vertex_to_facet(1);
    B.end_facet();

    B.end_surface();
  }
};

geometry_msgs::Point pointMsg(int x, int y, int z) {
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

TEST(CGALConversionsTest, triangle_mesh_to_msg) {
  Polyhedron m;
  TestingMesh<HalfedgeDS> testcase;
  m.delegate(testcase);

  EXPECT_TRUE(m.is_valid());

  // get message from mesh
  cgal_msgs::TriangleMesh msg;
  triangleMeshToMsg(&m, &msg);
  EXPECT_EQ(msg.vertices.size(), 4) << msg;
}

TEST(CGALConversionsTest, msg_to_triangle_mesh) {
  Polyhedron m1;
  Polyhedron m2;
  TestingMesh<HalfedgeDS> testcase;
  m1.delegate(testcase);
  EXPECT_TRUE(m1.is_valid());
  cgal_msgs::TriangleMesh msg;
  triangleMeshToMsg(&m1, &msg);
  msgToTriangleMesh(&msg, &m2);
  EXPECT_TRUE(m2.is_valid());
  
  //compare number of facets
  int i = 0;
  for (Polyhedron::Facet_iterator facet = m1.facets_begin();
        facet != m1.facets_end(); ++facet){
    ++i;
  }
  int j = 0;
  for (Polyhedron::Facet_iterator facet = m2.facets_begin();
        facet != m2.facets_end(); ++facet){
    ++j;
  }
  EXPECT_TRUE(i == j);

  //compare vertices of every triangle
  //this is a bit cumbersome since Polyhedrones can not be compared
  std::vector<int> vertices1;
  std::vector<int> vertices2;
  for (Polyhedron::Facet_iterator facet = m1.facets_begin();
        facet != m1.facets_end(); ++facet){
    Polyhedron::Halfedge_around_facet_const_circulator hit = facet->facet_begin();
    do {
    Point p = hit->vertex()->point();
    vertices1.push_back(p.x());
    vertices1.push_back(p.y());
    vertices1.push_back(p.z());
    } while (++hit != facet->facet_begin());
  }
  for (Polyhedron::Facet_iterator facet = m2.facets_begin();
      facet != m2.facets_end(); ++facet){
    Polyhedron::Halfedge_around_facet_const_circulator hit = facet->facet_begin();
    do {
    Point p = hit->vertex()->point();
    vertices2.push_back(p.x());
    vertices2.push_back(p.y());
    vertices2.push_back(p.z());
    } while (++hit != facet->facet_begin());
  }
  EXPECT_TRUE(vertices1 == vertices2);
}
