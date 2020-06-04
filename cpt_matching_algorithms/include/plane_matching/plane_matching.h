#ifndef PLANE_MATCHING_H_
#define PLANE_MATCHING_H_

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <ros/ros.h>
#include <Eigen/Eigenvalues>
#include "test_matcher/bounded_planes.h"

using namespace cad_percept::cgal;

namespace cad_percept {
namespace matching_algorithms {

class PlaneMatch {
 public:
  // Apply PRRUS matching on the found scan planes and the map planes, returns T_map,lidar
  static float PRRUS(float (&transformTR)[7], const pcl::PointCloud<pcl::PointNormal> scan_planes,
                     BoundedPlanes map_planes);

 private:
  // Calculate translation and find translational error between map and scan planes
  static void getTranslationError(Eigen::Matrix<int, 2, Eigen::Dynamic> assignment,
                                  float (&actual_transformation)[7], float &transl_error,
                                  Eigen::Quaternionf rotation,
                                  const pcl::PointCloud<pcl::PointNormal> scan_planes,
                                  BoundedPlanes map_planes, float translation_penalty);
};
}  // namespace matching_algorithms
}  // namespace cad_percept

#endif