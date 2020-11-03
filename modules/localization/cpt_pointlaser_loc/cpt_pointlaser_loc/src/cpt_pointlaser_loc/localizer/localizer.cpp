#include "cpt_pointlaser_loc/localizer/localizer.h"

#include <cgal_definitions/cgal_typedefs.h>
#include <cpt_utils/cpt_utils.h>
#include <glog/logging.h>
#include <kindr/minimal/rotation-quaternion.h>

namespace cad_percept {
namespace pointlaser_loc {
namespace localizer {
PointLaserLocalizer::PointLaserLocalizer(
    const cad_percept::cgal::MeshModel::Ptr &model,
    const Eigen::Matrix<double, 6, 1> &initial_armbase_to_ref_link_std,
    const Eigen::Matrix<double, 6, 1> &odometry_noise_std, double pointlaser_noise_std)
    : model_(model),
      initial_armbase_to_ref_link_std_(initial_armbase_to_ref_link_std),
      odometry_noise_std_(odometry_noise_std),
      pointlaser_noise_std_(pointlaser_noise_std),
      was_new_odometry_received_(false),
      were_new_laser_measurements_received_(false) {}

bool PointLaserLocalizer::setUpOptimizer(
    const kindr::minimal::QuatTransformation &marker_to_armbase,
    const kindr::minimal::QuatTransformation &initial_armbase_to_ref_link,
    const kindr::minimal::QuatTransformation &laser_a_offset,
    const kindr::minimal::QuatTransformation &laser_b_offset,
    const kindr::minimal::QuatTransformation &laser_c_offset,
    const kindr::minimal::QuatTransformation &endeffector_offset,
    const kindr::minimal::QuatTransformation &arm_base_to_base, bool fix_cad_planes,
    bool add_marker_pose_prior, bool only_optimize_translation) {
  // NOTE: we assume that the arm was already moved to the initial pose.

  // Instantiate an optimizer with the initial poses.
  optimizer_.reset(new cad_percept::pointlaser_loc::optimizer::LocalizationOptimizer(
      marker_to_armbase, initial_armbase_to_ref_link, model_, initial_armbase_to_ref_link_std_,
      odometry_noise_std_, pointlaser_noise_std_, fix_cad_planes, add_marker_pose_prior,
      only_optimize_translation));

  // Store the initial pose, required when adding measurements.
  initial_armbase_to_ref_link_.reset(
      new kindr::minimal::QuatTransformation(initial_armbase_to_ref_link));

  // Store the offset between the marker and the arm base.
  marker_to_armbase_.reset(new kindr::minimal::QuatTransformation(marker_to_armbase));

  // Store the laser offsets.
  laser_a_offset_.reset(new kindr::minimal::QuatTransformation(laser_a_offset));
  laser_b_offset_.reset(new kindr::minimal::QuatTransformation(laser_b_offset));
  laser_c_offset_.reset(new kindr::minimal::QuatTransformation(laser_c_offset));

  // Store the initial goal pose of the arm.
  arm_goal_pose_.reset(new kindr::minimal::QuatTransformation(
      arm_base_to_base.inverse() * initial_armbase_to_ref_link * endeffector_offset));
}

kindr::minimal::QuatTransformation PointLaserLocalizer::getArmGoalPose(
    Eigen::Quaternion<double> rotation_quat, kindr::minimal::PositionTemplate<double> translation) {
  CHECK(arm_goal_pose_ != nullptr) << "Must set up optimizer before getting arm goal pose.";
  rotation_quat.normalize();
  kindr::minimal::RotationQuaternionTemplate<double> rotation(rotation_quat);
  kindr::minimal::PositionTemplate<double> no_translation(0, 0, 0);
  kindr::minimal::RotationQuaternionTemplate<double> no_rotation(Eigen::Vector3d(0, 0, 0));

  kindr::minimal::QuatTransformation orientation_shift(no_translation, rotation);
  kindr::minimal::QuatTransformation position_shift(translation, no_rotation);

  *arm_goal_pose_ = position_shift * *arm_goal_pose_ * orientation_shift;

  return *arm_goal_pose_;
}

void PointLaserLocalizer::addOdometry(
    const kindr::minimal::QuatTransformation &odometry_transform) {
  CHECK(optimizer_ != nullptr) << "Must set up optimizer before adding odometry transform.";
  optimizer_->addOdometry(odometry_transform);

  was_new_odometry_received_ = true;
}

void PointLaserLocalizer::addLaserMeasurements(uint32_t distance_a, uint32_t distance_b,
                                               uint32_t distance_c) {
  CHECK(optimizer_ != nullptr && laser_a_offset_ != nullptr && laser_b_offset_ != nullptr &&
        laser_c_offset_ != nullptr)
      << "Must set up optimizer before adding laser measurements.";
  // It is assumed that the sensors measure distance in 1/10 mm.
  optimizer_->addRelativeMeasurement(distance_a / 10000.0, *laser_a_offset_);
  optimizer_->addRelativeMeasurement(distance_b / 10000.0, *laser_b_offset_);
  optimizer_->addRelativeMeasurement(distance_c / 10000.0, *laser_c_offset_);
}

void PointLaserLocalizer::getIntersectionsLasersWithModel(
    const kindr::minimal::QuatTransformation &current_arm_pose,
    cad_percept::cgal::Intersection *intersection_a,
    cad_percept::cgal::Intersection *intersection_b,
    cad_percept::cgal::Intersection *intersection_c) {
  CHECK_NOTNULL(intersection_a);
  CHECK_NOTNULL(intersection_b);
  CHECK_NOTNULL(intersection_c);

  CHECK(marker_to_armbase_ != nullptr && laser_a_offset_ != nullptr && laser_b_offset_ != nullptr &&
        laser_c_offset_ != nullptr)
      << "Must set up optimizer before getting intersections of lasers with model.";

  cad_percept::cgal::Ray query_ray_a = cad_percept::cpt_utils::buildRayFromPose(
      *marker_to_armbase_ * current_arm_pose * *laser_a_offset_);
  *intersection_a = model_->getIntersection(query_ray_a);

  cad_percept::cgal::Ray query_ray_b = cad_percept::cpt_utils::buildRayFromPose(
      *marker_to_armbase_ * current_arm_pose * *laser_b_offset_);
  *intersection_b = model_->getIntersection(query_ray_b);

  cad_percept::cgal::Ray query_ray_c = cad_percept::cpt_utils::buildRayFromPose(
      *marker_to_armbase_ * current_arm_pose * *laser_c_offset_);
  *intersection_c = model_->getIntersection(query_ray_c);

  were_new_laser_measurements_received_ = true;
}

kindr::minimal::QuatTransformation PointLaserLocalizer::optimizeForArmBasePoseInMap(bool verbose) {
  CHECK(optimizer_ != nullptr) << "Optimizer is not set up.";
  CHECK(was_new_odometry_received_ && were_new_laser_measurements_received_)
      << "Did not receive new odometry and/or laser measurements since initialization/last "
         "optimization.";

  // Reset the flags about the reception of measurements, in case a new optimization was to be
  // performed.
  was_new_odometry_received_ = false;
  were_new_laser_measurements_received_ = false;

  return optimizer_->optimize(verbose);
}

}  // namespace localizer
}  // namespace pointlaser_loc
}  // namespace cad_percept