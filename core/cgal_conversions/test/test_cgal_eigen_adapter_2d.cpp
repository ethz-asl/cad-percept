#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <cgal_conversions/cgal_eigen_adapter.h>
#include <cgal_conversions/eigen_conversions.h>
#include <cgal_definitions/cgal_typedefs.h>

using namespace cad_percept::cgal;

// Const test values (have to be equal to each other)
//  we use this values to return a specific type and later compare if it still has the same value
//  after conversion to another type.
Eigen::Vector2d EIGEN_VEC_VAL_2D(0.1, 0.2);
const Vector_2 CGAL_VEC_VAL_2D(0.1, 0.2);
const Point_2 CGAL_PT_VAL_2D(0.1, 0.2);

// Tests for Vector2Out
void assignFromEigenVector2d(Vector2Out output) { output = EIGEN_VEC_VAL_2D; }
void assignFromCGALVector2d(Vector2Out output) { output = CGAL_VEC_VAL_2D; }
void assignFromCGALPoint2d(Vector2Out output) { output = CGAL_PT_VAL_2D; }

// Test assignements from all types to Eigen Vector
TEST(CGALEigenAdapterTest2d, assignementsToEigenVector) {
  Eigen::Vector2d fromEigentoEigen(0.0, 0.0);
  Eigen::Vector2d fromCGALVectoEigen(0.0, 0.0);
  Eigen::Vector2d fromCGALPttoEigen(0.0, 0.0);
  assignFromEigenVector2d(&fromEigentoEigen);
  assignFromCGALVector2d(&fromCGALVectoEigen);
  assignFromCGALPoint2d(&fromCGALPttoEigen);
  ASSERT_EQ(fromEigentoEigen, EIGEN_VEC_VAL_2D);
  ASSERT_EQ(fromCGALVectoEigen, EIGEN_VEC_VAL_2D);
  ASSERT_EQ(fromCGALPttoEigen, EIGEN_VEC_VAL_2D);
}

// Test assignements from all types to CGAL Vector
TEST(CGALEigenAdapterTest2d, assignementsToCGALVector) {
  Vector_2 fromEigentoCGALVec(0.0, 0.0);
  Vector_2 fromCGALVectoCGALVec(0.0, 0.0);
  Vector_2 fromCGALPttoCGALVec(0.0, 0.0);
  assignFromEigenVector2d(&fromEigentoCGALVec);
  assignFromCGALVector2d(&fromCGALVectoCGALVec);
  assignFromCGALPoint2d(&fromCGALPttoCGALVec);
  ASSERT_EQ(fromEigentoCGALVec, CGAL_VEC_VAL_2D);
  ASSERT_EQ(fromCGALVectoCGALVec, CGAL_VEC_VAL_2D);
  ASSERT_EQ(fromCGALPttoCGALVec, CGAL_VEC_VAL_2D);
}

// Test assignements from all types to CGAL Point
TEST(CGALEigenAdapterTest2d, assignementsToCGALPoint) {
  Point_2 fromEigentoCGALPt(0.0, 0.0);
  Point_2 fromCGALVectoCGALPt(0.0, 0.0);
  Point_2 fromCGALPttoCGALPt(0.0, 0.0);
  assignFromEigenVector2d(&fromEigentoCGALPt);
  assignFromCGALVector2d(&fromCGALVectoCGALPt);
  assignFromCGALPoint2d(&fromCGALPttoCGALPt);
  ASSERT_EQ(fromEigentoCGALPt, CGAL_PT_VAL_2D);
  ASSERT_EQ(fromCGALVectoCGALPt, CGAL_PT_VAL_2D);
  ASSERT_EQ(fromCGALPttoCGALPt, CGAL_PT_VAL_2D);
}

// Tests for Vector2In
// This tests if the casts and inits for Vector2In work correctly.
Eigen::Vector2d returnToEigenVector2d(Vector2In input) { return input; }
Vector_2 returnToCGALVector2d(Vector2In input) { return input; }
Point_2 returnToCGALPoint2d(Vector2In input) { return input; }

// Test input params from all types to eigen return
TEST(CGALEigenAdapterTest2d, returnGenericAsEigen) {
  Eigen::Vector2d fromEigentoEigen(0.0, 0.0);
  Eigen::Vector2d fromCGALVectoEigen(0.0, 0.0);
  Eigen::Vector2d fromCGALPttoEigen(0.0, 0.0);
  fromEigentoEigen = returnToEigenVector2d(EIGEN_VEC_VAL_2D);
  fromCGALVectoEigen = returnToEigenVector2d(CGAL_VEC_VAL_2D);
  fromCGALPttoEigen = returnToEigenVector2d(CGAL_PT_VAL_2D);
  ASSERT_EQ(fromEigentoEigen, EIGEN_VEC_VAL_2D);
  ASSERT_EQ(fromCGALVectoEigen, EIGEN_VEC_VAL_2D);
  ASSERT_EQ(fromCGALPttoEigen, EIGEN_VEC_VAL_2D);
}

// Test input params from all types to CGAL Vector_2 return
TEST(CGALEigenAdapterTest2d, returnGenericAsCGALVector) {
  Vector_2 fromEigentoCGALVec(0.0, 0.0);
  Vector_2 fromCGALVectoCGALVec(0.0, 0.0);
  Vector_2 fromCGALPttoCGALVec(0.0, 0.0);
  fromEigentoCGALVec = returnToCGALVector2d(EIGEN_VEC_VAL_2D);
  fromCGALVectoCGALVec = returnToCGALVector2d(CGAL_VEC_VAL_2D);
  fromCGALPttoCGALVec = returnToCGALVector2d(CGAL_PT_VAL_2D);
  ASSERT_EQ(fromEigentoCGALVec, CGAL_VEC_VAL_2D);
  ASSERT_EQ(fromCGALVectoCGALVec, CGAL_VEC_VAL_2D);
  ASSERT_EQ(fromCGALPttoCGALVec, CGAL_VEC_VAL_2D);
}

// Test input params from all types to CGAL Point_2 return
TEST(CGALEigenAdapterTest2d, returnGenericAsCGALPoint) {
  Point_2 fromEigentoCGALPt(0.0, 0.0);
  Point_2 fromCGALVectoCGALPt(0.0, 0.0);
  Point_2 fromCGALPttoCGALPt(0.0, 0.0);
  fromEigentoCGALPt = returnToCGALPoint2d(EIGEN_VEC_VAL_2D);
  fromCGALVectoCGALPt = returnToCGALPoint2d(CGAL_VEC_VAL_2D);
  fromCGALPttoCGALPt = returnToCGALPoint2d(CGAL_PT_VAL_2D);
  ASSERT_EQ(fromEigentoCGALPt, CGAL_PT_VAL_2D);
  ASSERT_EQ(fromCGALVectoCGALPt, CGAL_PT_VAL_2D);
  ASSERT_EQ(fromCGALPttoCGALPt, CGAL_PT_VAL_2D);
}

// Tests for Vector2Return
Vector2Return returnFromEigenVector2d() { return EIGEN_VEC_VAL_2D; }
Vector2Return returnFromCGALVector2d() { return CGAL_VEC_VAL_2D; }
Vector2Return returnFromCGALPoint2d() { return CGAL_PT_VAL_2D; }

// Test returns to all types to Eigen Vector
TEST(CGALEigenAdapterTest2d, returnAsGenericInterpretAsEigen) {
  Eigen::Vector2d fromEigentoEigen(0.0, 0.0);
  Eigen::Vector2d fromCGALVectoEigen(0.0, 0.0);
  Eigen::Vector2d fromCGALPttoEigen(0.0, 0.0);
  fromEigentoEigen = returnFromEigenVector2d();
  fromCGALVectoEigen = returnFromCGALVector2d();
  fromCGALPttoEigen = returnFromCGALPoint2d();
  ASSERT_EQ(fromEigentoEigen, EIGEN_VEC_VAL_2D);
  ASSERT_EQ(fromCGALVectoEigen, EIGEN_VEC_VAL_2D);
  ASSERT_EQ(fromCGALPttoEigen, EIGEN_VEC_VAL_2D);
}

// Test returns to all types to CGAL Vector
TEST(CGALEigenAdapterTest2d, returnAsGenericInterpretAsCGALVector) {
  Vector_2 fromEigentoCGALVec(0.0, 0.0);
  Vector_2 fromCGALVectoCGALVec(0.0, 0.0);
  Vector_2 fromCGALPttoCGALVec(0.0, 0.0);
  fromEigentoCGALVec = returnFromEigenVector2d();
  fromCGALVectoCGALVec = returnFromCGALVector2d();
  fromCGALPttoCGALVec = returnFromCGALPoint2d();
  ASSERT_EQ(fromEigentoCGALVec, CGAL_VEC_VAL_2D);
  ASSERT_EQ(fromCGALVectoCGALVec, CGAL_VEC_VAL_2D);
  ASSERT_EQ(fromCGALPttoCGALVec, CGAL_VEC_VAL_2D);
}

// Test returns to all types to CGAL Point
TEST(CGALEigenAdapterTest2d, returnAsGenericInterpretAsCGALPoint) {
  Point_2 fromEigentoCGALPt(0.0, 0.0);
  Point_2 fromCGALVectoCGALPt(0.0, 0.0);
  Point_2 fromCGALPttoCGALPt(0.0, 0.0);
  fromEigentoCGALPt = returnFromEigenVector2d();
  fromCGALVectoCGALPt = returnFromCGALVector2d();
  fromCGALPttoCGALPt = returnFromCGALPoint2d();
  ASSERT_EQ(fromEigentoCGALPt, CGAL_PT_VAL_2D);
  ASSERT_EQ(fromCGALVectoCGALPt, CGAL_PT_VAL_2D);
  ASSERT_EQ(fromCGALPttoCGALPt, CGAL_PT_VAL_2D);
}