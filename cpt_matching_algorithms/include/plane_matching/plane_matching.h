// PRRUS describes an algorithm for matching extracted planes from a LiDAR scan to the planes of a
// map. First, sets are created consisting of three planes whose normals are orthogonal to each
// other. This is done for the scan and the map. Then two of the three reference plane normals are
// used to derive the rotation between the scan and the map. The third reference plane can then be
// used to validate the obtained rotation, in order to discard certain assignments early on. In case
// that no such orthogonal sets could be found or no assignment could fulfill the rotation
// consistency condition, the algorithm returns an error message. For the remaining ones, the
// translation is now determined and a cost function is evaluated, which evaluates the quality of
// the assignment based on the remaining planes. The cost function is based on the translational
// distance of the map plane along the map plane normal to the closest scan plane. Certain
// conditions must be met so that a scan plane can be assigned to a map plane. If no possible
// assignment is found, this indicates an incorrect assignment and will be penalized with a penalty.
// The assignment with the lowest value is assumed to be correct.

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
  // Loads Config for RHT from Server
  struct prrusConfig {
    float translation_penalty;
  };
  static prrusConfig loadPrrusConfigFromServer();

  // Apply PRRUS matching on the found scan planes and the map planes, returns T_map,lidar
  static float prrus(Transformation &transform, const pcl::PointCloud<pcl::PointNormal> scan_planes,
                     BoundedPlanes map_planes, const prrusConfig &config);

 private:
  // Find orthogonal sets of planes in planes and add them to orthogonal_groups, if add_permutation
  // assign permutation of found groups
  static void createOrthogonalSets(std::vector<Eigen::Vector3i> &orthogonal_groups,
                                   const pcl::PointCloud<pcl::PointNormal> &planes,
                                   bool add_permutation);
  // Returns assingments, which result in a consistent rotation and returns corresponding rotation
  static void checkRotationConsistency(
      std::vector<std::vector<std::pair<size_t, size_t>>> &assignments,
      std::vector<Eigen::Quaterniond> &rotations_of_assignments,
      const std::vector<Eigen::Vector3i> &orthogonal_groups_in_scan,
      const std::vector<Eigen::Vector3i> &orthogonal_groups_in_map,
      const pcl::PointCloud<pcl::PointNormal> &scan_planes,
      const pcl::PointCloud<pcl::PointNormal> &map_planes);
  // Calculate translation and find translational error between map and scan planes
  static void getTranslationError(std::vector<std::pair<size_t, size_t>> plane_assignment,
                                  Transformation &transform, float &transl_error,
                                  Eigen::Quaterniond rotation,
                                  const pcl::PointCloud<pcl::PointNormal> &scan_planes,
                                  BoundedPlanes map_planes, float translation_penalty);
  // Helper functions
  static Eigen::Vector3d PointToEigenVector(pcl::PointNormal point);
  static Eigen::Vector3d NormalToEigenVector(pcl::PointNormal point);
};
}  // namespace matching_algorithms
}  // namespace cad_percept

#endif