#include <gflags/gflags.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cgal_conversions/eigen_conversions.h>
#include <cgal_definitions/cgal_typedefs.h>

using namespace cad_percept::cgal;

TEST(CGALConversionsTest, vector_to_eigen_vector) {
  //convert in both directions and compare
  Eigen::Vector3d v_eigen(4,8,3);
  Vector v_cgal = eigenVectorToVector(v_eigen);
  Eigen::Vector3d v_eigen_comp = vectorToEigenVector(v_cgal);

  EXPECT_TRUE(v_eigen == v_eigen_comp);
}
