#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <cgal_definitions/mesh_model.h>
#include <cgal_definitions/cgal_typedefs.h>

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

TEST(CGALMeshModelTest, normal_computation) {
  Polyhedron m;
  TestingMesh<HalfedgeDS> testcase;
  m.delegate(testcase);

  EXPECT_TRUE(m.is_valid());

  MeshModel model(m);
  Polyhedron P = model.getMesh(); 

  // Comparing the 4 different methods
  Polyhedron::Facet_handle fh = P.facets_begin(); // use new Polyhedron since we access new model
  fh++;
  fh++;
  Vector test_normal1;  
  Vector test_normal2;  
  Vector test_normal3;  
  Vector test_normal4;  
  test_normal1 = model.getNormal(fh);
  std::cout << "Normal with getNormal is: " << test_normal1 << std::endl;
  test_normal2 = model.computeFaceNormal(fh);
  std::cout << "Normal with descriptor is: " << test_normal2 << std::endl;
  test_normal3 = model.computeFaceNormal2(fh);
  std::cout << "Normal with cross product is: " << test_normal3 << std::endl;
  std::map<int, Vector> normals_map = model.computeNormals();
  test_normal4 = normals_map[fh->id()];
  std::cout << "Normal in map is: " << test_normal4 << std::endl;

  EXPECT_TRUE(test_normal1 == test_normal2 && test_normal2 == test_normal3 && test_normal3 == test_normal4);

  // Polyhedron can save plane equation with operator:
  // https://doc.cgal.org/latest/Polyhedron/Polyhedron_2polyhedron_prog_normals_8cpp-example.html
}
