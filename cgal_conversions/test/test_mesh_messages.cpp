#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <gflags/gflags.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cgal_conversions/mesh_conversions.h>
#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_msgs/TriangleMesh.h>

using namespace cad_percept::cgal;

typedef Polyhedron::HalfedgeDS HalfedgeDS;

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

TEST(CGALConversionsTest, traingle_mesh_to_msg) {
  Polyhedron m;
  TestingMesh<HalfedgeDS> testcase;
  m.delegate(testcase);

  EXPECT_TRUE(m.is_valid());

  // get message from mesh
  cgal_msgs::TriangleMesh msg;
  triangleMeshToMsg(&m, &msg);
  EXPECT_EQ(msg.vertices.size(), 4) << msg;
}

// TODO Conversion tests cgal->msg-> cgal
