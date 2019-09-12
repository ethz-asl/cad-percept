#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <cgal_conversions/cgal_eigen_adapter.h>
#include <cgal_conversions/eigen_conversions.h>
#include <cgal_definitions/cgal_typedefs.h>

using namespace cad_percept::cgal;

// Const test values (have to be equal to each other)
//  we use this values to return a specific type and later compare if it still has the same value
//  after conversion to another type.
const Eigen::Vector3d EIGEN_VEC_VAL(0.1, 0.2, 0.3);
const Vector CGAL_VEC_VAL(0.1, 0.2, 0.3);
const Point CGAL_PT_VAL(0.1, 0.2, 0.3);

// Tests for Vector3Out
void assignFromEigenVector3d(Vector3Out output) { output = EIGEN_VEC_VAL; }
void assignFromCGALVector3d(Vector3Out output) { output = CGAL_VEC_VAL; }
void assignFromCGALPoint3d(Vector3Out output) { output = CGAL_PT_VAL; }

// Test assignements from all types to Eigen Vector
TEST(CGALEigenAdapterTest3d, assignementsToEigenVector) {
  Eigen::Vector3d fromEigentoEigen(0.0, 0.0, 0.0);
  Eigen::Vector3d fromCGALVectoEigen(0.0, 0.0, 0.0);
  Eigen::Vector3d fromCGALPttoEigen(0.0, 0.0, 0.0);
  assignFromEigenVector3d(&fromEigentoEigen);
  assignFromCGALVector3d(&fromCGALVectoEigen);
  assignFromCGALPoint3d(&fromCGALPttoEigen);
  ASSERT_EQ(fromEigentoEigen, EIGEN_VEC_VAL);
  ASSERT_EQ(fromCGALVectoEigen, EIGEN_VEC_VAL);
  ASSERT_EQ(fromCGALPttoEigen, EIGEN_VEC_VAL);
}

// Test assignements from all types to CGAL Vector
TEST(CGALEigenAdapterTest3d, assignementsToCGALVector) {
  Vector fromEigentoCGALVec(0.0, 0.0, 0.0);
  Vector fromCGALVectoCGALVec(0.0, 0.0, 0.0);
  Vector fromCGALPttoCGALVec(0.0, 0.0, 0.0);
  assignFromEigenVector3d(&fromEigentoCGALVec);
  assignFromCGALVector3d(&fromCGALVectoCGALVec);
  assignFromCGALPoint3d(&fromCGALPttoCGALVec);
  ASSERT_EQ(fromEigentoCGALVec, CGAL_VEC_VAL);
  ASSERT_EQ(fromCGALVectoCGALVec, CGAL_VEC_VAL);
  ASSERT_EQ(fromCGALPttoCGALVec, CGAL_VEC_VAL);
}

// Test assignements from all types to CGAL Point
TEST(CGALEigenAdapterTest3d, assignementsToCGALPoint) {
  Point fromEigentoCGALPt(0.0, 0.0, 0.0);
  Point fromCGALVectoCGALPt(0.0, 0.0, 0.0);
  Point fromCGALPttoCGALPt(0.0, 0.0, 0.0);
  assignFromEigenVector3d(&fromEigentoCGALPt);
  assignFromCGALVector3d(&fromCGALVectoCGALPt);
  assignFromCGALPoint3d(&fromCGALPttoCGALPt);
  ASSERT_EQ(fromEigentoCGALPt, CGAL_PT_VAL);
  ASSERT_EQ(fromCGALVectoCGALPt, CGAL_PT_VAL);
  ASSERT_EQ(fromCGALPttoCGALPt, CGAL_PT_VAL);
}

// Tests for Vector3In
// This tests if the casts and inits for Vector3In work correctly.
Eigen::Vector3d returnToEigenVector3d(Vector3In input) { return input; }
Vector returnToCGALVector3d(Vector3In input) { return input; }
Point returnToCGALPoint3d(Vector3In input) { return input; }

// Test input params from all types to eigen return
TEST(CGALEigenAdapterTest3d, returnGenericAsEigen) {
  Eigen::Vector3d fromEigentoEigen(0.0, 0.0, 0.0);
  Eigen::Vector3d fromCGALVectoEigen(0.0, 0.0, 0.0);
  Eigen::Vector3d fromCGALPttoEigen(0.0, 0.0, 0.0);
  fromEigentoEigen = returnToEigenVector3d(EIGEN_VEC_VAL);
  fromCGALVectoEigen = returnToEigenVector3d(CGAL_VEC_VAL);
  fromCGALPttoEigen = returnToEigenVector3d(CGAL_PT_VAL);
  ASSERT_EQ(fromEigentoEigen, EIGEN_VEC_VAL);
  ASSERT_EQ(fromCGALVectoEigen, EIGEN_VEC_VAL);
  ASSERT_EQ(fromCGALPttoEigen, EIGEN_VEC_VAL);
}

// Test input params from all types to CGAL Vector return
TEST(CGALEigenAdapterTest3d, returnGenericAsCGALVector) {
  Vector fromEigentoCGALVec(0.0, 0.0, 0.0);
  Vector fromCGALVectoCGALVec(0.0, 0.0, 0.0);
  Vector fromCGALPttoCGALVec(0.0, 0.0, 0.0);
  fromEigentoCGALVec = returnToCGALVector3d(EIGEN_VEC_VAL);
  fromCGALVectoCGALVec = returnToCGALVector3d(CGAL_VEC_VAL);
  fromCGALPttoCGALVec = returnToCGALVector3d(CGAL_PT_VAL);
  ASSERT_EQ(fromEigentoCGALVec, CGAL_VEC_VAL);
  ASSERT_EQ(fromCGALVectoCGALVec, CGAL_VEC_VAL);
  ASSERT_EQ(fromCGALPttoCGALVec, CGAL_VEC_VAL);
}

// Test input params from all types to CGAL Point return
TEST(CGALEigenAdapterTest3d, returnGenericAsCGALPoint) {
  Point fromEigentoCGALPt(0.0, 0.0, 0.0);
  Point fromCGALVectoCGALPt(0.0, 0.0, 0.0);
  Point fromCGALPttoCGALPt(0.0, 0.0, 0.0);
  fromEigentoCGALPt = returnToCGALPoint3d(EIGEN_VEC_VAL);
  fromCGALVectoCGALPt = returnToCGALPoint3d(CGAL_VEC_VAL);
  fromCGALPttoCGALPt = returnToCGALPoint3d(CGAL_PT_VAL);
  ASSERT_EQ(fromEigentoCGALPt, CGAL_PT_VAL);
  ASSERT_EQ(fromCGALVectoCGALPt, CGAL_PT_VAL);
  ASSERT_EQ(fromCGALPttoCGALPt, CGAL_PT_VAL);
}

// Tests for Vextor3Return
Vector3Return returnFromEigenVector3d() { return EIGEN_VEC_VAL; }
Vector3Return returnFromCGALVector3d() { return CGAL_VEC_VAL; }
Vector3Return returnFromCGALPoint3d() { return CGAL_PT_VAL; }

// Test returns to all types to Eigen Vector
TEST(CGALEigenAdapterTest3d, returnAsGenericInterpretAsEigen) {
  Eigen::Vector3d fromEigentoEigen(0.0, 0.0, 0.0);
  Eigen::Vector3d fromCGALVectoEigen(0.0, 0.0, 0.0);
  Eigen::Vector3d fromCGALPttoEigen(0.0, 0.0, 0.0);
  fromEigentoEigen = returnFromEigenVector3d();
  fromCGALVectoEigen = returnFromCGALVector3d();
  fromCGALPttoEigen = returnFromCGALPoint3d();
  ASSERT_EQ(fromEigentoEigen, EIGEN_VEC_VAL);
  ASSERT_EQ(fromCGALVectoEigen, EIGEN_VEC_VAL);
  ASSERT_EQ(fromCGALPttoEigen, EIGEN_VEC_VAL);
}

// Test returns to all types to CGAL Vector
TEST(CGALEigenAdapterTest3d, returnAsGenericInterpretAsCGALVector) {
  Vector fromEigentoCGALVec(0.0, 0.0, 0.0);
  Vector fromCGALVectoCGALVec(0.0, 0.0, 0.0);
  Vector fromCGALPttoCGALVec(0.0, 0.0, 0.0);
  fromEigentoCGALVec = returnFromEigenVector3d();
  fromCGALVectoCGALVec = returnFromCGALVector3d();
  fromCGALPttoCGALVec = returnFromCGALPoint3d();
  ASSERT_EQ(fromEigentoCGALVec, CGAL_VEC_VAL);
  ASSERT_EQ(fromCGALVectoCGALVec, CGAL_VEC_VAL);
  ASSERT_EQ(fromCGALPttoCGALVec, CGAL_VEC_VAL);
}

// Test returns to all types to CGAL Point
TEST(CGALEigenAdapterTest3d, returnAsGenericInterpretAsCGALPoint) {
  Point fromEigentoCGALPt(0.0, 0.0, 0.0);
  Point fromCGALVectoCGALPt(0.0, 0.0, 0.0);
  Point fromCGALPttoCGALPt(0.0, 0.0, 0.0);
  fromEigentoCGALPt = returnFromEigenVector3d();
  fromCGALVectoCGALPt = returnFromCGALVector3d();
  fromCGALPttoCGALPt = returnFromCGALPoint3d();
  ASSERT_EQ(fromEigentoCGALPt, CGAL_PT_VAL);
  ASSERT_EQ(fromCGALVectoCGALPt, CGAL_PT_VAL);
  ASSERT_EQ(fromCGALPttoCGALPt, CGAL_PT_VAL);
}