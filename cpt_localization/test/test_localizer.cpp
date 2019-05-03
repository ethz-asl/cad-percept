#include <glog/logging.h>
#include <gtest/gtest.h>

#include "cpt_localization/mesh_localizer.h"

using namespace cad_percept::localization;

void ASSERT_MATRIX_EQ(const Eigen::MatrixXd &A,
                      const Eigen::MatrixXd &B,
                      std::string str) {
  double tol = 1e-7;
  ASSERT_EQ(A.rows(), B.rows()) << str << " are not the same size.";
  ASSERT_EQ(A.cols(), B.cols()) << str << " are not the same size.";
  for (int i = 0; i < A.rows(); i++) {
    for (int j = 0; j < A.cols(); j++) {
      ASSERT_NEAR(A(i, j), B(i, j), tol) << str << " have different values.";
    }
  }
}

// Initialize common objects needed by multiple tests.
class LocalizationTest : public ::testing::Test {
 protected:
  cad_percept::localization::MeshLocalizer mesh_localizer_;
  PointCloud point_cloud_;
  LocalizationTest() {
    // Initiate simple tetrahedron mesh.
    cad_percept::cgal::SurfaceMesh sample_mesh;
    cad_percept::cgal::Point vertex_a(-1.0, 1.0, 10.0);
    cad_percept::cgal::Point vertex_b(-1.0, 1.0, -1.0);
    cad_percept::cgal::Point vertex_c(10.0, 1.0, -1.0);
    cad_percept::cgal::Point vertex_d(-1.0, -10.0, -1.0) ;
    sample_mesh.make_tetrahedron(vertex_a, vertex_b, vertex_d, vertex_c);
    mesh_localizer_.setMesh(sample_mesh);

    // Make point cloud.
    pcl::PointXYZ point_a, point_b, point_c, point_d, point_e, point_f;
    point_a.x = -0.9;
    point_a.y = 0.5;
    point_a.z = 0.0;
    point_b.x = 0.0;
    point_b.y = 0.9;
    point_b.z = 0.0;
    point_c.x = 0.0;
    point_c.y = 0.0;
    point_c.z = -0.9;
    point_d.x = -0.9;
    point_d.y = 0.6;
    point_d.z = 0.0;
    point_e.x = 0.0;
    point_e.y = 0.9;
    point_e.z = 0.1;
    point_f.x = 0.1;
    point_f.y = 0.0;
    point_f.z = -0.9;
    point_cloud_.height = 1;
    point_cloud_.width = 6;
    point_cloud_.resize(point_cloud_.height * point_cloud_.width);
    point_cloud_.points[0] = point_a;
    point_cloud_.points[1] = point_b;
    point_cloud_.points[2] = point_c;
    point_cloud_.points[3] = point_d;
    point_cloud_.points[4] = point_e;
    point_cloud_.points[5] = point_f;
  }
};

TEST_F(LocalizationTest, test_localization) {
  SE3 initial_pose = SE3(SE3::Position(0, 0, 0), SE3::Rotation(1, 0, 0, 0));
  SE3 result = mesh_localizer_.icm(point_cloud_, initial_pose);
  ASSERT_MATRIX_EQ(result.getPosition(), Eigen::Vector3d(-0.1, 0.1, -0.1),
      "localization result");
}
