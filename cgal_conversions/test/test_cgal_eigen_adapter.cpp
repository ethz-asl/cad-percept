#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <cgal_conversions/cgal_eigen_adapter.h>
#include <cgal_conversions/eigen_conversions.h>
#include <cgal_definitions/cgal_typedefs.h>

using namespace cad_percept::cgal;

// Const test values (have to be equal to each other)
const Eigen::Vector3d EVAL(0.1, 0.2, 0.3);
const Vector CVAL(0.1, 0.2, 0.3);

// Tests for Vector3Out
void testVectorOutEigenAssignment(Vector3Out output) { output = EVAL; }
void testVectorOutCGALAssignment(Vector3Out output) { output = CVAL; }

// Test all 4 combinations
TEST(CGALEigenAdapterTest, eigenVectorOutEigenAssignment) {
  Eigen::Vector3d test(0.0, 0.0, 0.0);
  testVectorOutEigenAssignment(&test);
  ASSERT_EQ(test, EVAL);
}

TEST(CGALEigenAdapterTest, cgalVectorOutEigenAssignment) {
  Vector test(0.0, 0.0, 0.0);
  testVectorOutEigenAssignment(&test);
  ASSERT_EQ(test, CVAL);
}

TEST(CGALEigenAdapterTest, eigenVectorOutCGALAssignment) {
  Eigen::Vector3d test(0.0, 0.0, 0.0);
  testVectorOutCGALAssignment(&test);
  ASSERT_EQ(test, EVAL);
}

TEST(CGALEigenAdapterTest, cgalVectorOutCGALAssignment) {
  Vector test(0.0, 0.0, 0.0);
  testVectorOutCGALAssignment(&test);
  ASSERT_EQ(test, CVAL);
}

// Tests for Vector3In
Eigen::Vector3d testVectorInEigenReturn(Vector3In input) { return input; }
Vector testVectorInCGALReturn(Vector3In input) { return input; }

TEST(CGALEigenAdapterTest, eigenVectorInEigenReturn) {
  Eigen::Vector3d test = testVectorInEigenReturn(EVAL);
  ASSERT_EQ(test, EVAL);
}

TEST(CGALEigenAdapterTest, cgalVectorInEigenReturn) {
  Eigen::Vector3d test = testVectorInEigenReturn(CVAL);
  ASSERT_EQ(test, EVAL);
}

TEST(CGALEigenAdapterTest, eigenVectorInCGALReturn) {
  Vector test = testVectorInCGALReturn(EVAL);
  ASSERT_EQ(test, CVAL);
}

TEST(CGALEigenAdapterTest, cgalVectorInCGALReturn) {
  Vector test = testVectorInCGALReturn(CVAL);
  ASSERT_EQ(test, CVAL);
}

// Tests for Vextor3Return
Vector3Return testVector3ReturnCGAL() { return CVAL; }
Vector3Return testVector3ReturnEigen() { return EVAL; }

TEST(CGALEigenAdapterTest, cgalVectorReturnCGAL) {
  Vector test = testVector3ReturnCGAL();
  ASSERT_EQ(test, CVAL);
}

TEST(CGALEigenAdapterTest, cgalVectorReturnEigen) {
  Vector test = testVector3ReturnEigen();
  ASSERT_EQ(test, CVAL);
}

TEST(CGALEigenAdapterTest, eigenVectorReturnCGAL) {
  Eigen::Vector3d test = testVector3ReturnCGAL();
  ASSERT_EQ(test, EVAL);
}

TEST(CGALEigenAdapterTest, eigenVectorReturnEigen) {
  Eigen::Vector3d test = testVector3ReturnEigen();
  ASSERT_EQ(test, EVAL);
}
