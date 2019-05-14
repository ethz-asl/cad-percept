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
  triangleMeshToMsg(m, &msg);
  EXPECT_EQ(msg.vertices.size(), 4) << msg;
}

TEST(CGALConversionsTest, msg_to_triangle_mesh) {
  Polyhedron m1;
  Polyhedron m2;
  TestingMesh<HalfedgeDS> testcase;
  m1.delegate(testcase);
  EXPECT_TRUE(m1.is_valid());
  cgal_msgs::TriangleMesh msg;
  triangleMeshToMsg(m1, &msg);
  msgToTriangleMesh(msg, &m2);
  EXPECT_TRUE(m2.is_valid());

  // compare number of facets
  EXPECT_TRUE(m1.size_of_facets() == m2.size_of_facets());

  // compare vertices of every triangle
  // this is a bit cumbersome since Polyhedrones can not be compared
  std::vector<int> vertices1;
  std::vector<int> vertices2;
  for (Polyhedron::Facet_iterator facet = m1.facets_begin();
       facet != m1.facets_end(); ++facet) {
    Polyhedron::Halfedge_around_facet_const_circulator hit =
        facet->facet_begin();
    do {
      Point p = hit->vertex()->point();
      vertices1.push_back(p.x());
      vertices1.push_back(p.y());
      vertices1.push_back(p.z());
    } while (++hit != facet->facet_begin());
  }
  for (Polyhedron::Facet_iterator facet = m2.facets_begin();
       facet != m2.facets_end(); ++facet) {
    Polyhedron::Halfedge_around_facet_const_circulator hit =
        facet->facet_begin();
    do {
      Point p = hit->vertex()->point();
      vertices2.push_back(p.x());
      vertices2.push_back(p.y());
      vertices2.push_back(p.z());
    } while (++hit != facet->facet_begin());
  }
  EXPECT_TRUE(vertices1 == vertices2);
}

TEST(CGALConversionsTest, mesh_to_vertice_point_cloud) {
  // generate test mesh
  Polyhedron m;
  TestingMesh<HalfedgeDS> testcase;
  m.delegate(testcase);
  EXPECT_TRUE(m.is_valid());

  PointCloud pc;
  meshToVerticePointCloud(m, &pc);

  // check number of points
  EXPECT_TRUE(m.size_of_vertices() == pc.size());

  // check coordinates of points
  // order of vertex points and in p.c. stays the same, so:
  std::vector<int> vertices;
  std::vector<int> points;

  for (auto vertex_point = m.points_begin(); vertex_point != m.points_end();
       ++vertex_point) {
    vertices.push_back(vertex_point->x());
    vertices.push_back(vertex_point->y());
    vertices.push_back(vertex_point->z());
  }

  for (auto pc_points : pc.points) {
    points.push_back(pc_points.x);
    points.push_back(pc_points.y);
    points.push_back(pc_points.z);
  }

  EXPECT_TRUE(vertices == points);
}

TEST(CGALConversionsTest, msg_conversions) {
  //test for functions: triToProbMsg, probToTriMsg, triangleMeshToProbMsg, probMsgToTriangleMesh
  
  //generate test mesh
  Polyhedron m;
  TestingMesh<HalfedgeDS> testcase;
  m.delegate(testcase);
  EXPECT_TRUE(m.is_valid());

  cgal_msgs::ProbabilisticMesh p_msg;
  cgal_msgs::TriangleMesh t_msg;
  Polyhedron m_comp;

  triangleMeshToProbMsg(m, &p_msg);
  probToTriMsg(p_msg, &t_msg);
  triToProbMsg(t_msg, &p_msg);
  probMsgToTriangleMesh(p_msg, &m_comp);

  //compare polyhedron
  EXPECT_TRUE(m_comp.is_valid());
  
  // compare number of facets
  EXPECT_TRUE(m.size_of_facets() == m_comp.size_of_facets());

  // compare vertices of every triangle
  // this is a bit cumbersome since Polyhedrones can not be compared
  std::vector<int> vertices1;
  std::vector<int> vertices2;
  for (Polyhedron::Facet_iterator facet = m.facets_begin();
       facet != m.facets_end(); ++facet) {
    Polyhedron::Halfedge_around_facet_const_circulator hit =
        facet->facet_begin();
    do {
      Point p = hit->vertex()->point();
      vertices1.push_back(p.x());
      vertices1.push_back(p.y());
      vertices1.push_back(p.z());
    } while (++hit != facet->facet_begin());
  }
  for (Polyhedron::Facet_iterator facet = m_comp.facets_begin();
       facet != m_comp.facets_end(); ++facet) {
    Polyhedron::Halfedge_around_facet_const_circulator hit =
        facet->facet_begin();
    do {
      Point p = hit->vertex()->point();
      vertices2.push_back(p.x());
      vertices2.push_back(p.y());
      vertices2.push_back(p.z());
    } while (++hit != facet->facet_begin());
  }
  EXPECT_TRUE(vertices1 == vertices2);
}
