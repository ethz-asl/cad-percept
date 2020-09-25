#include "localization_optimizer/localization_optimizer.h"

namespace cad_percept {
namespace localization_optimizer {

Eigen::Matrix<double, 6, 1> getIntersectionPlaneImplementation(
    const kindr::minimal::QuatTransformation& sensor_pose,
    const std::shared_ptr<architect_model::ArchitectModel> model,
    gtsam::OptionalJacobian<6, 6> H, gtsam::OptionalJacobian<6, 1> H_ignored) {
  if (H) *H = Eigen::Matrix<double, 6, 6>::Zero();
  if (H_ignored) {
    // We know that H_ignored is never provided, because the function works only
    // if model is a constant.
    *H_ignored = Eigen::Matrix<double, 6, 1>::Zero();
  }
  architect_model::Intersection i = model->getIntersection(sensor_pose);
  Eigen::Matrix<double, 6, 1> response;
  response.head<3>() = i.point;
  response.tail<3>() = i.surface_normal;
  return response;
};

gtsam::Expression<Eigen::Matrix<double, 6, 1>> getIntersectionPlane(
    ETransformation& sensor_pose,
    gtsam::Expression<std::shared_ptr<architect_model::ArchitectModel>>&
       
        model) {
  return gtsam::Expression<Eigen::Matrix<double, 6, 1>>(
      &getIntersectionPlaneImplementation, sensor_pose, model);
};

// Get the expected distance for the retrieved plane and the laser pose.
gtsam::Expression<double> expectedDistance(ETransformation& laser_in_map,
                                           EVector3& plane_support,
                                           EVector3& plane_normal) {
  EVector3 unit_dir(Eigen::Vector3d(1, 0, 0));
  EVector3 origin(Eigen::Vector3d(0, 0, 0));
  // construct ray
  EVector3 ray_direction =
      rotate(rotationFromTransformation(laser_in_map), unit_dir);
  EVector3 ray_origin = laser_in_map * origin;
  // find intersection distance
  gtsam::Expression<double> shortest_distance =
      multiplyVectors(plane_support - ray_origin, plane_normal);
  return checkPositive(
      divide(shortest_distance, multiplyVectors(ray_direction, plane_normal)));
};

LocalizationOptimizer::LocalizationOptimizer(
    const kindr::minimal::QuatTransformation& architecture_offset,
    const kindr::minimal::QuatTransformation& initial_pose,
    const std::shared_ptr<architect_model::ArchitectModel> arch_model,
    const Eigen::Matrix<double, 6, 1>& architecture_offset_std,
    const Eigen::Matrix<double, 6, 1>& odometry_noise_std,
    const double pointlaser_noise_std, const bool fix_retrieved_planes,
    const bool add_prior, const bool only_optimize_translation)
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
          gtsam::noiseModel::Diagonal::Sigmas(Vector1(pointlaser_noise_std)))),
      fix_retrieved_planes_(fix_retrieved_planes),
      only_optimize_translation_(only_optimize_translation) {
  // Initialize the optimizer with the pose that is measured by the state
  // estimator.
  initialization_.insert<kindr::minimal::QuatTransformation>(
      0, architecture_offset);
  if (add_prior) {
    // add a factor for the prior of the architecture_offset
    graph_.addExpressionFactor(
        architect_offset_, architecture_offset,
        gtsam::noiseModel::Diagonal::Sigmas(architecture_offset_std));
  }
};

Eigen::Vector3d LocalizationOptimizer::addRelativeMeasurement(
    const double distance,
    const kindr::minimal::QuatTransformation joint2sensor) {
  ETransformation armbase2sensor(current_arm_pose_ * joint2sensor);
  // TODO once cpp14 is supported, change to unique pointers
  std::shared_ptr<ETransformation> laser_in_map;
  std::shared_ptr<EVector3> plane_normal, plane_support;
  if (only_optimize_translation_) {
    ETransformation fixed_architect_offset(initial_architect_offset_);
    ETransformation architect_offset_with_fixed_rot =
        transformationFromComponents(
            rotationFromTransformation(fixed_architect_offset),
            translationFromTransformation(architect_offset_));
    laser_in_map = std::make_shared<ETransformation>(
        architect_offset_with_fixed_rot * armbase2sensor);
  } else {
    laser_in_map =
        std::make_shared<ETransformation>(architect_offset_ * armbase2sensor);
  }
  if (fix_retrieved_planes_) {
    kindr::minimal::QuatTransformation slam_guess_laser_in_map =
        initial_architect_offset_ * current_arm_pose_ * joint2sensor;
    architect_model::Intersection plane =
        architect_model_->getIntersection(slam_guess_laser_in_map);
    plane_support = std::make_shared<EVector3>(plane.point);
    plane_normal = std::make_shared<EVector3>(plane.surface_normal);
  } else {
    // make a constant from the architecture model
    auto model =
        gtsam::Expression<std::shared_ptr<architect_model::ArchitectModel>>(
            architect_model_);
    auto intersection = getIntersectionPlane(*laser_in_map, model);
    plane_normal =
        std::make_shared<EVector3>(getIntersectionNormal(intersection));
    plane_support =
        std::make_shared<EVector3>(getIntersectionPoint(intersection));
  }
  EVector3 unit_dir(Eigen::Vector3d(1, 0, 0));
  EVector3 origin(Eigen::Vector3d(0, 0, 0));
  // construct ray
  EVector3 ray_direction =
      rotate(rotationFromTransformation(*laser_in_map), unit_dir);
  EVector3 ray_origin = *laser_in_map * origin;
  // find intersection distance
  gtsam::Expression<double> shortest_distance =
      multiplyVectors(*plane_support - ray_origin, *plane_normal);
  gtsam::Expression<double> expected_distance = checkPositive(
      divide(shortest_distance, multiplyVectors(ray_direction, *plane_normal)));
  graph_.addExpressionFactor(expected_distance, distance, pointlaser_noise_);
  // Return the intersection point for debugging / visualization.
  // Caution: for constant expressions, argument of value function is ignored.
  return plane_support->value(initialization_);
};

void LocalizationOptimizer::addOdometry(
    const kindr::minimal::QuatTransformation transform) {
  current_arm_pose_ = current_arm_pose_ * transform;
};

kindr::minimal::QuatTransformation LocalizationOptimizer::optimize(
    const bool verbose) {
  // for debugging
  if (verbose) graph_.print("\nlocalisation graph:\n");
  if (verbose) initialization_.print("initializations:\n");
  LevenbergMarquardtParams opt_params;
  if (verbose) opt_params.setVerbosityLM("TRYLAMBDA");
  Values result =
      LevenbergMarquardtOptimizer(graph_, initialization_, opt_params)
          .optimize();
  if (only_optimize_translation_) {
    // the rotation is still somehow changed in the optimization, but for the
    // loss always fixed to the initialization. Overwrite it again with the
    // initialization.
    kindr::minimal::QuatTransformation optimized(
        result.at<kindr::minimal::QuatTransformation>(0));
    kindr::minimal::QuatTransformation ret(
        optimized.getPosition(), initial_architect_offset_.getRotation());
    return ret;
  }
  return result.at<kindr::minimal::QuatTransformation>(0);
};
}  // namespace localization_optimizer
}  // namespace cad_percept
