#ifndef LOCALIZATION_OPTIMIZER
#define LOCALIZATION_OPTIMIZER

#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/expressions.h>
#include <kindr/minimal/common-gtsam.h>
#include <kindr/minimal/cubic-hermite-interpolation-gtsam.h>
#include <kindr/minimal/cubic-hermite-quaternion-gtsam.h>
#include <kindr/minimal/quat-transformation-gtsam.h>
#include <kindr/minimal/quat-transformation.h>
#include <kindr/minimal/rotation-quaternion-gtsam.h>

#include <Eigen/Geometry>

#include "pointlaser_loc/architectmodel.h"
#include "pointlaser_loc/architectmodel_fake_trait.h"

namespace localization_optimizer {

using namespace gtsam;
using namespace kindr::minimal;

typedef gtsam::Expression<QuatTransformation> ETransformation;
typedef gtsam::Expression<Eigen::Vector3d> EVector3;

static double multiplyVectorsImplementation(Eigen::Vector3d a,
                                            Eigen::Vector3d b,
                                            gtsam::OptionalJacobian<1, 3> Ha,
                                            gtsam::OptionalJacobian<1, 3> Hb) {
  if (Ha) *Ha = b.transpose();
  if (Hb) *Hb = a.transpose();
  return a.transpose() * b;
}

static Expression<double> multiplyVectors(
    const Expression<Eigen::Vector3d>& C1,
    const Expression<Eigen::Vector3d>& C2) {
  return Expression<double>(&multiplyVectorsImplementation, C1, C2);
}

static double checkPositiveImplementation(double a,
                                          gtsam::OptionalJacobian<1, 1> H) {
  if (H) (*H)(0, 0) = 1.0;
  if (a < 0) std::cout << "WARNING: input is negative" << std::endl;
  return a;
}

static Expression<double> checkPositive(const Expression<double>& C) {
  return Expression<double>(&checkPositiveImplementation, C);
}

static double divideImplementation(double a, double b,
                                   OptionalJacobian<1, 1> Ha,
                                   OptionalJacobian<1, 1> Hb) {
  if (Ha) (*Ha)(0, 0) = 1.0 / b;
  if (Hb) (*Hb)(0, 0) = -a / (b * b);
  return a / b;
}

static Expression<double> divide(const Expression<double>& a,
                                 const Expression<double>& b) {
  return Expression<double>(&divideImplementation, a, b);
}

static Eigen::Vector3d getIntersectionPointImplementation(
    const Eigen::Matrix<double, 6, 1>& intersection, OptionalJacobian<3, 6> H) {
  if (H) *H = Eigen::Matrix<double, 3, 6>::Identity();
  return intersection.head<3>();
}

static EVector3 getIntersectionPoint(
    const Expression<Eigen::Matrix<double, 6, 1>>& intersection) {
  return EVector3(&getIntersectionPointImplementation, intersection);
}

static Eigen::Vector3d getIntersectionNormalImplementation(
    const Eigen::Matrix<double, 6, 1>& intersection, OptionalJacobian<3, 6> H) {
  if (H) {
    H->leftCols<3>().setZero();
    H->rightCols<3>().setIdentity();
  }
  return intersection.tail<3>();
}

static EVector3 getIntersectionNormal(
    const Expression<Eigen::Matrix<double, 6, 1>>& intersection) {
  return EVector3(&getIntersectionNormalImplementation, intersection);
}

Eigen::Matrix<double, 6, 1> getIntersectionPlaneImplementation(
    const QuatTransformation& sensor_pose,
    const std::shared_ptr<architect_model::ArchitectModel> model,
    gtsam::OptionalJacobian<6, 6> H, gtsam::OptionalJacobian<6, 1> H_ignored);

Expression<Eigen::Matrix<double, 6, 1>> getIntersectionPlane(
    const ETransformation& sensor_pose,
    Expression<std::shared_ptr<architect_model::ArchitectModel>>& model);

Expression<double> expectedDistance(ETransformation& laser_in_map,
                                    EVector3& plane_support,
                                    EVector3& plane_normal);

class LocalizationOptimizer {
 public:
  LocalizationOptimizer(
      const QuatTransformation& architecture_offset,
      const QuatTransformation& initial_pose,
      const std::shared_ptr<architect_model::ArchitectModel> arch_model,
      const Eigen::Matrix<double, 6, 1>& architecture_offset_std,
      const Eigen::Matrix<double, 6, 1>& odometry_noise_std,
      const double pointlaser_noise_std, const bool fix_retrieved_planes,
      const bool add_prior, const bool only_optimize_translation);
  Eigen::Vector3d addRelativeMeasurement(const double distance,
                                         const QuatTransformation joint2sensor);
  void addOdometry(const QuatTransformation odometry);
  QuatTransformation optimize(const bool verbose = false);

 private:
  bool fix_retrieved_planes_, only_optimize_translation_;
  ExpressionFactorGraph graph_;
  QuatTransformation current_arm_pose_, initial_architect_offset_;
  ETransformation architect_offset_;
  std::shared_ptr<architect_model::ArchitectModel> architect_model_;
  noiseModel::Base::shared_ptr pointlaser_noise_;
  noiseModel::Diagonal::shared_ptr odometry_noise_;
  Values initialization_;
};
}  // namespace localization_optimizer
#endif
