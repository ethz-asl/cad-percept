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
    const kindr::minimal::QuatTransformation &laser_c_offset, bool fix_cad_planes,
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
}

void PointLaserLocalizer::addOdometry(
    const kindr::minimal::QuatTransformation &odometry_transform) {
  CHECK(optimizer_ != nullptr) << "Must set up optimizer before adding odometry transform.";
  optimizer_->addOdometry(odometry_transform);

  was_new_odometry_received_ = true;
}

void PointLaserLocalizer::addLaserMeasurements(
    uint32_t distance_a, uint32_t distance_b, uint32_t distance_c,
    std::vector<Eigen::Vector3d> *intersected_plane_normals,
    std::vector<Eigen::Vector3d> *intersected_plane_supports,
    std::vector<std::string> *intersected_face_ids) {
  CHECK(optimizer_ != nullptr && laser_a_offset_ != nullptr && laser_b_offset_ != nullptr &&
        laser_c_offset_ != nullptr)
      << "Must set up optimizer before adding laser measurements.";
  // It is assumed that the sensors measure distance in 1/10 mm.
  ROS_INFO_STREAM("Adding the following relative measurement for laser A: " << distance_a /
                                                                                   10000.0);
  ROS_INFO_STREAM("Adding the following relative measurement for laser B: " << distance_b /
                                                                                   10000.0);
  ROS_INFO_STREAM("Adding the following relative measurement for laser C: " << distance_c / 10000.0
                                                                            << "\n");
  std::vector<Eigen::Vector3d> intersected_plane_normals_, intersected_plane_supports_;
  std::vector<std::string> intersected_face_ids_;
  intersected_plane_normals_.resize(3);
  intersected_plane_supports_.resize(3);
  intersected_face_ids_.resize(3);
  optimizer_->addRelativeMeasurement(distance_a / 10000.0, *laser_a_offset_,
                                     &intersected_plane_normals_[0],
                                     &intersected_plane_supports_[0], &intersected_face_ids_[0]);
  optimizer_->addRelativeMeasurement(distance_b / 10000.0, *laser_b_offset_,
                                     &intersected_plane_normals_[1],
                                     &intersected_plane_supports_[1], &intersected_face_ids_[1]);
  optimizer_->addRelativeMeasurement(distance_c / 10000.0, *laser_c_offset_,
                                     &intersected_plane_normals_[2],
                                     &intersected_plane_supports_[2], &intersected_face_ids_[2]);
  if (intersected_plane_normals != nullptr) {
    *intersected_plane_normals = intersected_plane_normals_;
  }
  if (intersected_plane_supports != nullptr) {
    *intersected_plane_supports = intersected_plane_supports_;
  }
  if (intersected_face_ids != nullptr) {
    *intersected_face_ids = intersected_face_ids_;
  }
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