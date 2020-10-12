#include "cpt_pointlaser_loc/optimizer/optimizer.h"

#include <cpt_utils/cpt_utils.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include "cpt_pointlaser_loc/optimizer/geometric_utils.h"

namespace cad_percept {
namespace pointlaser_loc {
namespace optimizer {

LocalizationOptimizer::LocalizationOptimizer(
    const kindr::minimal::QuatTransformation &architecture_offset,
    const kindr::minimal::QuatTransformation &initial_pose,
    const cad_percept::cgal::MeshModel::Ptr &arch_model,
    const Eigen::Matrix<double, 6, 1> &architecture_offset_std,
    const Eigen::Matrix<double, 6, 1> &odometry_noise_std, const double pointlaser_noise_std,
    const bool fix_retrieved_planes, const bool add_prior, const bool only_optimize_translation)
    : initialization_(),
      graph_(),
      architect_offset_(0),
      initial_architect_offset_(architecture_offset),
      current_arm_pose_(initial_pose),
      architect_model_(arch_model),
      odometry_noise_(gtsam::noiseModel::Diagonal::Sigmas(odometry_noise_std)),
      // could also try GemanMcClure
      pointlaser_noise_(gtsam::noiseModel::Robust::Create(
          gtsam::noiseModel::mEstimator::Cauchy::Create(1),
          gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(pointlaser_noise_std)))),
      fix_retrieved_planes_(fix_retrieved_planes),
      only_optimize_translation_(only_optimize_translation) {
  // Initialize the optimizer with the pose that is measured by the state
  // estimator.
  initialization_.insert<kindr::minimal::QuatTransformation>(0, architecture_offset);
  if (add_prior) {
    // add a factor for the prior of the architecture_offset
    graph_.addExpressionFactor(architect_offset_, architecture_offset,
                               gtsam::noiseModel::Diagonal::Sigmas(architecture_offset_std));
  }
};

Eigen::Vector3d LocalizationOptimizer::addRelativeMeasurement(
    const double distance, const kindr::minimal::QuatTransformation &joint2sensor) {
  ETransformation armbase2sensor(current_arm_pose_ * joint2sensor);
  // TODO once cpp14 is supported, change to unique pointers
  std::shared_ptr<ETransformation> laser_in_map;
  std::shared_ptr<EVector3> plane_normal, plane_support;
  if (only_optimize_translation_) {
    ETransformation fixed_architect_offset(initial_architect_offset_);
    ETransformation architect_offset_with_fixed_rot =
        transformationFromComponents(rotationFromTransformation(fixed_architect_offset),
                                     translationFromTransformation(architect_offset_));
    laser_in_map =
        std::make_shared<ETransformation>(architect_offset_with_fixed_rot * armbase2sensor);
  } else {
    laser_in_map = std::make_shared<ETransformation>(architect_offset_ * armbase2sensor);
  }
  if (fix_retrieved_planes_) {
    kindr::minimal::QuatTransformation slam_guess_laser_in_map =
        initial_architect_offset_ * current_arm_pose_ * joint2sensor;
    // Build the ray to query for intersections.
    cad_percept::cgal::Ray query_ray =
        cad_percept::cpt_utils::buildRayFromPose(slam_guess_laser_in_map);
    cad_percept::cgal::Intersection plane = architect_model_->getIntersection(query_ray);
    plane_support = std::make_shared<EVector3>(Eigen::Vector3d(
        plane.intersected_point.x(), plane.intersected_point.y(), plane.intersected_point.z()));
    plane_normal = std::make_shared<EVector3>(Eigen::Vector3d(
        plane.surface_normal.x(), plane.surface_normal.y(), plane.surface_normal.z()));
  } else {
    // make a constant from the architecture model
    auto model = gtsam::Expression<cad_percept::cgal::MeshModel::Ptr>(architect_model_);
    auto intersection = getIntersectionPlane(*laser_in_map, model);
    plane_normal = std::make_shared<EVector3>(getIntersectionNormal(intersection));
    plane_support = std::make_shared<EVector3>(getIntersectionPoint(intersection));
  }
  EVector3 unit_dir(Eigen::Vector3d(1, 0, 0));
  EVector3 origin(Eigen::Vector3d(0, 0, 0));
  // construct ray
  EVector3 ray_direction = rotate(rotationFromTransformation(*laser_in_map), unit_dir);
  EVector3 ray_origin = *laser_in_map * origin;
  // find intersection distance
  gtsam::Expression<double> shortest_distance =
      multiplyVectors(*plane_support - ray_origin, *plane_normal);
  gtsam::Expression<double> expected_distance =
      checkPositive(divide(shortest_distance, multiplyVectors(ray_direction, *plane_normal)));
  graph_.addExpressionFactor(expected_distance, distance, pointlaser_noise_);
  // Return the intersection point for debugging / visualization.
  // Caution: for constant expressions, argument of value function is ignored.
  return plane_support->value(initialization_);
};

void LocalizationOptimizer::addOdometry(const kindr::minimal::QuatTransformation &transform) {
  current_arm_pose_ = current_arm_pose_ * transform;
};

kindr::minimal::QuatTransformation LocalizationOptimizer::optimize(const bool verbose) {
  // for debugging
  if (verbose) graph_.print("\nlocalisation graph:\n");
  if (verbose) initialization_.print("initializations:\n");
  gtsam::LevenbergMarquardtParams opt_params;
  if (verbose) opt_params.setVerbosityLM("TRYLAMBDA");
  gtsam::Values result =
      gtsam::LevenbergMarquardtOptimizer(graph_, initialization_, opt_params).optimize();
  if (only_optimize_translation_) {
    // the rotation is still somehow changed in the optimization, but for the
    // loss always fixed to the initialization. Overwrite it again with the
    // initialization.
    kindr::minimal::QuatTransformation optimized(result.at<kindr::minimal::QuatTransformation>(0));
    kindr::minimal::QuatTransformation ret(optimized.getPosition(),
                                           initial_architect_offset_.getRotation());
    return ret;
  }
  return result.at<kindr::minimal::QuatTransformation>(0);
};

}  // namespace optimizer
}  // namespace pointlaser_loc
}  // namespace cad_percept
