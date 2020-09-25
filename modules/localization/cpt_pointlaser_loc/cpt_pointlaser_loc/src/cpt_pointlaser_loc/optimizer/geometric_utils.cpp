#include "cpt_pointlaser_loc/optimizer/geometric_utils.h"

namespace cad_percept {
namespace pointlaser_loc {
namespace optimizer {

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
}

gtsam::Expression<Eigen::Matrix<double, 6, 1>> getIntersectionPlane(
    ETransformation& sensor_pose,
    gtsam::Expression<std::shared_ptr<architect_model::ArchitectModel>>&

        model) {
  return gtsam::Expression<Eigen::Matrix<double, 6, 1>>(
      &getIntersectionPlaneImplementation, sensor_pose, model);
}

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
}

}  // namespace optimizer
}  // namespace pointlaser_loc
}  // namespace cad_percept
