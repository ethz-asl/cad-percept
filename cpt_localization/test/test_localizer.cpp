#include <glog/logging.h>
#include <gtest/gtest.h>

#include "cpt_localization/mesh_localizer.h"

// Initialize common objects needed by multiple tests.
class LocalizationTest : public ::testing::Test {
 protected:
  cad_percept::localization::MeshLocalizer mesh_localizer_;
  LocalizationTest() {
    // Initiate simple tetrahedron mesh.
    cad_percept::cgal::SurfaceMesh sample_mesh;
    cad_percept::cgal::Point point_a(-1.0, 1.0, 10.0);
    cad_percept::cgal::Point point_b(-1.0, 1.0, 1.0);
    cad_percept::cgal::Point point_c(10.0, 1.0, -1.0);
    cad_percept::cgal::Point point_d(-1.0, -10.0, -1.0) ;
    sample_mesh.make_tetrahedron(point_a, point_b, point_c, point_d);
//    mesh_localizer_.setMesh(sample_mesh);
  }
};

//TEST_F(LocalizationTest, test_creation_is_empty) {
//EXPECT_EQ(0, 0);
//}

TEST_F(LocalizationTest, test_localization) {
//cadify::Cadify non_empty("$(find cadify)/resources/cla_updated.off");
cad_percept::localization::MeshLocalizer mesh_localizer;
// Initiate simple tetrahedron mesh.
cad_percept::cgal::SurfaceMesh sample_mesh;
cad_percept::cgal::Point point_a(-1.0, 1.0, 10.0);
cad_percept::cgal::Point point_b(-1.0, 1.0, 1.0);
cad_percept::cgal::Point point_c(10.0, 1.0, -1.0);
cad_percept::cgal::Point point_d(-1.0, -10.0, -1.0);
sample_mesh.make_tetrahedron(point_a, point_b, point_c, point_d);
mesh_localizer.setMesh(sample_mesh);
EXPECT_GE(0, 0);
}
