#include <gflags/gflags.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cgal_conversions/eigen_conversions.h>
#include <cgal_definitions/cgal_typedefs.h>

using namespace cad_percept::cgal;

TEST(CGALConversionsTest, vector_to_eigen_vector) {
  //convert in both directions and compare
  Eigen::Vector3d v_eigen_a(Eigen::Vector3d::Random());
  Vector v_cgal_a = eigenVectorToVector(v_eigen_a);
  Eigen::Vector3d v_eigen_comp_a = vectorToEigenVector(v_cgal_a);

  EXPECT_TRUE(v_eigen_a == v_eigen_comp_a);

  Eigen::Vector3d v_eigen_b(Eigen::Vector3d::Random());
  Vector v_cgal_b;
  eigenVectorToVector(&v_eigen_b, &v_cgal_b);
  Eigen::Vector3d v_eigen_comp_b;
  vectorToEigenVector(&v_cgal_b, &v_eigen_comp_b);

  EXPECT_TRUE(v_eigen_b == v_eigen_comp_b);
}

TEST(CGALConversionsTest, point_to_eigen_vector) {
  //convert in both directions and compare
  Eigen::Vector3d v_eigen_a(Eigen::Vector3d::Random());
  Point p_cgal_a = eigenVectorToCgalPoint(v_eigen_a);
  Eigen::Vector3d v_eigen_comp_a = cgalPointToEigenVector(p_cgal_a);

  EXPECT_TRUE(v_eigen_a == v_eigen_comp_a);

  Eigen::Vector3d v_eigen_b(Eigen::Vector3d::Random());
  Point p_cgal_b;
  eigenVectorToCgalPoint(&v_eigen_b, &p_cgal_b);
  Eigen::Vector3d v_eigen_comp_b;
  cgalPointToEigenVector(&p_cgal_b, &v_eigen_comp_b);

  EXPECT_TRUE(v_eigen_b == v_eigen_comp_b);
}