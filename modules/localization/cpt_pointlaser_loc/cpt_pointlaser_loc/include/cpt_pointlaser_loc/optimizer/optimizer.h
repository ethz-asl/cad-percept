#ifndef LOCALIZATION_OPTIMIZER_LOCALIZATION_OPTIMIZER_H_
#define LOCALIZATION_OPTIMIZER_LOCALIZATION_OPTIMIZER_H_

#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <kindr/minimal/quat-transformation.h>

#include <Eigen/Geometry>

#include "cpt_pointlaser_loc/optimizer/common.h"
#include "pointlaser_loc/architectmodel.h"
#include "pointlaser_loc/architectmodel_fake_trait.h"

namespace cad_percept {
namespace pointlaser_loc {
namespace optimizer {

static double multiplyVectorsImplementation(Eigen::Vector3d a,
                                            Eigen::Vector3d b,
                                            gtsam::OptionalJacobian<1, 3> Ha,
                                            gtsam::OptionalJacobian<1, 3> Hb) {
  if (Ha) *Ha = b.transpose();
  if (Hb) *Hb = a.transpose();
  return a.transpose() * b;
}

static gtsam::Expression<double> multiplyVectors(
    const gtsam::Expression<Eigen::Vector3d>& C1,
    const gtsam::Expression<Eigen::Vector3d>& C2) {
  return gtsam::Expression<double>(&multiplyVectorsImplementation, C1, C2);
}

static double checkPositiveImplementation(double a,
                                          gtsam::OptionalJacobian<1, 1> H) {
  if (H) (*H)(0, 0) = 1.0;
  if (a < 0) std::cout << "WARNING: input is negative" << std::endl;
  return a;
}

static gtsam::Expression<double> checkPositive(
    const gtsam::Expression<double>& C) {
  return gtsam::Expression<double>(&checkPositiveImplementation, C);
}

static double divideImplementation(double a, double b,
                                   gtsam::OptionalJacobian<1, 1> Ha,
                                   gtsam::OptionalJacobian<1, 1> Hb) {
  if (Ha) (*Ha)(0, 0) = 1.0 / b;
  if (Hb) (*Hb)(0, 0) = -a / (b * b);
  return a / b;
}

static gtsam::Expression<double> divide(const gtsam::Expression<double>& a,
                                        const gtsam::Expression<double>& b) {
  return gtsam::Expression<double>(&divideImplementation, a, b);
}

static Eigen::Vector3d getIntersectionPointImplementation(
    const Eigen::Matrix<double, 6, 1>& intersection,
    gtsam::OptionalJacobian<3, 6> H) {
  if (H) *H = Eigen::Matrix<double, 3, 6>::Identity();
  return intersection.head<3>();
}

static EVector3 getIntersectionPoint(
    const gtsam::Expression<Eigen::Matrix<double, 6, 1>>& intersection) {
  return EVector3(&getIntersectionPointImplementation, intersection);
}

static Eigen::Vector3d getIntersectionNormalImplementation(
    const Eigen::Matrix<double, 6, 1>& intersection,
    gtsam::OptionalJacobian<3, 6> H) {
  if (H) {
    H->leftCols<3>().setZero();
    H->rightCols<3>().setIdentity();
  }
  return intersection.tail<3>();
}

static EVector3 getIntersectionNormal(
    const gtsam::Expression<Eigen::Matrix<double, 6, 1>>& intersection) {
  return EVector3(&getIntersectionNormalImplementation, intersection);
}

Eigen::Matrix<double, 6, 1> getIntersectionPlaneImplementation(
    const kindr::minimal::QuatTransformation& sensor_pose,
    const std::shared_ptr<architect_model::ArchitectModel> model,
    gtsam::OptionalJacobian<6, 6> H, gtsam::OptionalJacobian<6, 1> H_ignored);

gtsam::Expression<Eigen::Matrix<double, 6, 1>> getIntersectionPlane(
    const ETransformation& sensor_pose,
    gtsam::Expression<std::shared_ptr<architect_model::ArchitectModel>>& model);

gtsam::Expression<double> expectedDistance(ETransformation& laser_in_map,
                                           EVector3& plane_support,
                                           EVector3& plane_normal);

class LocalizationOptimizer {
 public:
  LocalizationOptimizer(
      const kindr::minimal::QuatTransformation& architecture_offset,
      const kindr::minimal::QuatTransformation& initial_pose,
      const std::shared_ptr<architect_model::ArchitectModel> arch_model,
      const Eigen::Matrix<double, 6, 1>& architecture_offset_std,
      const Eigen::Matrix<double, 6, 1>& odometry_noise_std,
      const double pointlaser_noise_std, const bool fix_retrieved_planes,
      const bool add_prior, const bool only_optimize_translation);
  Eigen::Vector3d addRelativeMeasurement(
      const double distance,
      const kindr::minimal::QuatTransformation joint2sensor);
  void addOdometry(const kindr::minimal::QuatTransformation odometry);
  kindr::minimal::QuatTransformation optimize(const bool verbose = false);

 private:
  bool fix_retrieved_planes_, only_optimize_translation_;
  gtsam::ExpressionFactorGraph graph_;
  kindr::minimal::QuatTransformation current_arm_pose_,
      initial_architect_offset_;
  ETransformation architect_offset_;
  std::shared_ptr<architect_model::ArchitectModel> architect_model_;
  gtsam::noiseModel::Base::shared_ptr pointlaser_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr odometry_noise_;
  gtsam::Values initialization_;
};
}  // namespace optimizer
}  // namespace pointlaser_loc
}  // namespace cad_percept
#endif  // LOCALIZATION_OPTIMIZER_LOCALIZATION_OPTIMIZER_H_
