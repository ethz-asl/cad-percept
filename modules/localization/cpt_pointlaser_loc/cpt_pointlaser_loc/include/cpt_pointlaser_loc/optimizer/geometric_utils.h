#ifndef CPT_POINTLASER_LOC_OPTIMIZER_GEOMETRIC_UTILS_H_
#define CPT_POINTLASER_LOC_OPTIMIZER_GEOMETRIC_UTILS_H_

#include <cgal_definitions/mesh_model.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/nonlinear/Expression.h>
#include <kindr/minimal/quat-transformation.h>

#include <Eigen/Geometry>

#include "cpt_pointlaser_loc/model/mesh-model-gtsam.h"
#include "cpt_pointlaser_loc/optimizer/common.h"

namespace cad_percept {
namespace pointlaser_loc {
namespace optimizer {

static double multiplyVectorsImplementation(Eigen::Vector3d a, Eigen::Vector3d b,
                                            gtsam::OptionalJacobian<1, 3> Ha,
                                            gtsam::OptionalJacobian<1, 3> Hb) {
  if (Ha) *Ha = b.transpose();
  if (Hb) *Hb = a.transpose();
  return a.transpose() * b;
}

static gtsam::Expression<double> multiplyVectors(const gtsam::Expression<Eigen::Vector3d> &C1,
                                                 const gtsam::Expression<Eigen::Vector3d> &C2) {
  return gtsam::Expression<double>(&multiplyVectorsImplementation, C1, C2);
}

static double checkPositiveImplementation(double a, gtsam::OptionalJacobian<1, 1> H) {
  if (H) (*H)(0, 0) = 1.0;
  if (a < 0) std::cout << "WARNING: input is negative" << std::endl;
  return a;
}

static gtsam::Expression<double> checkPositive(const gtsam::Expression<double> &C) {
  return gtsam::Expression<double>(&checkPositiveImplementation, C);
}

static double divideImplementation(double a, double b, gtsam::OptionalJacobian<1, 1> Ha,
                                   gtsam::OptionalJacobian<1, 1> Hb) {
  if (Ha) (*Ha)(0, 0) = 1.0 / b;
  if (Hb) (*Hb)(0, 0) = -a / (b * b);
  return a / b;
}

static gtsam::Expression<double> divide(const gtsam::Expression<double> &a,
                                        const gtsam::Expression<double> &b) {
  return gtsam::Expression<double>(&divideImplementation, a, b);
}

static Eigen::Vector3d getIntersectionPointImplementation(
    const Eigen::Matrix<double, 6, 1> &intersection, gtsam::OptionalJacobian<3, 6> H) {
  if (H) *H = Eigen::Matrix<double, 3, 6>::Identity();
  return intersection.head<3>();
}

static EVector3 getIntersectionPoint(
    const gtsam::Expression<Eigen::Matrix<double, 6, 1>> &intersection) {
  return EVector3(&getIntersectionPointImplementation, intersection);
}

static Eigen::Vector3d getIntersectionNormalImplementation(
    const Eigen::Matrix<double, 6, 1> &intersection, gtsam::OptionalJacobian<3, 6> H) {
  if (H) {
    H->leftCols<3>().setZero();
    H->rightCols<3>().setIdentity();
  }
  return intersection.tail<3>();
}

static EVector3 getIntersectionNormal(
    const gtsam::Expression<Eigen::Matrix<double, 6, 1>> &intersection) {
  return EVector3(&getIntersectionNormalImplementation, intersection);
}

Eigen::Matrix<double, 6, 1> getIntersectionPlaneImplementation(
    const kindr::minimal::QuatTransformation &sensor_pose,
    const cad_percept::cgal::MeshModel::Ptr model, gtsam::OptionalJacobian<6, 6> H,
    gtsam::OptionalJacobian<6, 1> H_ignored);

gtsam::Expression<Eigen::Matrix<double, 6, 1>> getIntersectionPlane(
    const ETransformation &sensor_pose,
    gtsam::Expression<cad_percept::cgal::MeshModel::Ptr> &model);

gtsam::Expression<double> expectedDistance(ETransformation &laser_in_map, EVector3 &plane_support,
                                           EVector3 &plane_normal);

}  // namespace optimizer
}  // namespace pointlaser_loc
}  // namespace cad_percept
#endif  // CPT_POINTLASER_LOC_OPTIMIZER_GEOMETRIC_UTILS_H_
