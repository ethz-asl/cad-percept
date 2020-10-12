#include "cpt_pointlaser_loc/optimizer/geometric_utils.h"

#include <cpt_utils/cpt_utils.h>

namespace cad_percept {
namespace pointlaser_loc {
namespace optimizer {

Eigen::Matrix<double, 6, 1> getIntersectionPlaneImplementation(
    const kindr::minimal::QuatTransformation &sensor_pose,
    const cad_percept::cgal::MeshModel::Ptr model, gtsam::OptionalJacobian<6, 6> H,
    gtsam::OptionalJacobian<6, 1> H_ignored) {
  if (H) *H = Eigen::Matrix<double, 6, 6>::Zero();
  if (H_ignored) {
    // We know that H_ignored is never provided, because the function works only
    // if model is a constant.
    *H_ignored = Eigen::Matrix<double, 6, 1>::Zero();
  }

  cad_percept::cgal::Ray query_ray = cad_percept::cpt_utils::buildRayFromPose(sensor_pose);
  cad_percept::cgal::Intersection i = model->getIntersection(query_ray);
  Eigen::Matrix<double, 6, 1> response;
  response.head<3>() =
      Eigen::Vector3d(i.intersected_point.x(), i.intersected_point.y(), i.intersected_point.z());
  response.tail<3>() =
      Eigen::Vector3d(i.surface_normal.x(), i.surface_normal.y(), i.surface_normal.z());
  return response;
}

gtsam::Expression<Eigen::Matrix<double, 6, 1>> getIntersectionPlane(
    const ETransformation &sensor_pose,
    gtsam::Expression<cad_percept::cgal::MeshModel::Ptr> &model) {
  return gtsam::Expression<Eigen::Matrix<double, 6, 1>>(&getIntersectionPlaneImplementation,
                                                        sensor_pose, model);
}

// Get the expected distance for the retrieved plane and the laser pose.
gtsam::Expression<double> expectedDistance(ETransformation &laser_in_map, EVector3 &plane_support,
                                           EVector3 &plane_normal) {
  EVector3 unit_dir(Eigen::Vector3d(1, 0, 0));
  EVector3 origin(Eigen::Vector3d(0, 0, 0));
  // construct ray
  EVector3 ray_direction = rotate(rotationFromTransformation(laser_in_map), unit_dir);
  EVector3 ray_origin = laser_in_map * origin;
  // find intersection distance
  gtsam::Expression<double> shortest_distance =
      multiplyVectors(plane_support - ray_origin, plane_normal);
  return checkPositive(divide(shortest_distance, multiplyVectors(ray_direction, plane_normal)));
}

}  // namespace optimizer
}  // namespace pointlaser_loc
}  // namespace cad_percept
