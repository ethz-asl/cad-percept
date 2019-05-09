#include <gflags/gflags.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cgal_conversions/eigen_conversions.h>
#include <cgal_definitions/cgal_typedefs.h>

using namespace cad_percept::cgal;

TEST(CGALConversionsTest, vector_to_vector) {
  //convert in both directions and compare
  Eigen::Vector3d v_eigen_a(Eigen::Vector3d::Random());
  Vector v_cgal_a = eigenVectorToCgalVector(v_eigen_a);
  Eigen::Vector3d v_eigen_comp_a = cgalVectorToEigenVector(v_cgal_a);

  EXPECT_TRUE(v_eigen_a == v_eigen_comp_a);

  Eigen::Vector3d v_eigen_b(Eigen::Vector3d::Random());
  Vector v_cgal_b;
  eigenVectorToCgalVector(&v_eigen_b, &v_cgal_b);
  Eigen::Vector3d v_eigen_comp_b;
  cgalVectorToEigenVector(&v_cgal_b, &v_eigen_comp_b);

  EXPECT_TRUE(v_eigen_b == v_eigen_comp_b);
}

TEST(CGALConversionsTest, point_to_vector) {
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

TEST(CGALConversionsTest, transformation_to_transformation) {
  //convert in both directions and compare
  Eigen::Matrix4d t_eigen_a(Eigen::Matrix4d::Random());
  t_eigen_a(3,0) = 0.0; // need to set these to get correct representation
  t_eigen_a(3,1) = 0.0;
  t_eigen_a(3,2) = 0.0;
  t_eigen_a(3,3) = 1.0;

  Transformation t_cgal_a = eigenTransformationToCgalTransformation(t_eigen_a);
  Eigen::Matrix4d t_eigen_comp_a = cgalTransformationToEigenTransformation(t_cgal_a);

  EXPECT_TRUE(t_eigen_a == t_eigen_comp_a);

  Eigen::Matrix4d t_eigen_b(Eigen::Matrix4d::Random());
  t_eigen_b(3,0) = 0.0; // need to set these to get correct representation
  t_eigen_b(3,1) = 0.0;
  t_eigen_b(3,2) = 0.0;
  t_eigen_b(3,3) = 1.0;

  Transformation t_cgal_b;
  eigenTransformationToCgalTransformation(&t_eigen_b, &t_cgal_b);
  Eigen::Matrix4d t_eigen_comp_b;
  cgalTransformationToEigenTransformation(&t_cgal_b, &t_eigen_comp_b);

  EXPECT_TRUE(t_eigen_b == t_eigen_comp_b);
}