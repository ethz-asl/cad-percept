#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <Eigen/Eigenvalues>

#include <cgal_conversions/eigen_conversions.h>
#include <cgal_definitions/cgal_typedefs.h>

#include "cloud_filter/cloud_filter.h"

// Remove this again
#include <cgal_conversions/mesh_conversions.h>
#include <cgal_conversions/tf_conversions.h>
#include <cgal_definitions/mesh_model.h>
#include <cgal_msgs/TriangleMeshStamped.h>
#include <cpt_utils/pc_processing.h>
#include <pcl/point_types.h>
#include <pointmatcher/PointMatcher.h>
#include <pointmatcher_ros/point_cloud.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

using namespace cad_percept::cgal;

namespace cad_percept {
namespace matching_algorithms {

class PlaneMatch {
 public:
  static void IntersectionPatternMatcher(float (&transformTR)[7],
                                         const pcl::PointCloud<pcl::PointNormal> scan_planes,
                                         const pcl::PointCloud<pcl::PointNormal> map_planes,
                                         Eigen::Matrix<float, 22, 2> room_boundaries);
  static void loadExampleSol(float (&transformTR)[7],
                             const pcl::PointCloud<pcl::PointNormal> scan_planes,
                             const pcl::PointCloud<pcl::PointNormal> map_planes);

 private:
  struct IntersectionCornernessPoint {
    Point intersection_point;
    float cornerness;
  };
  class SortRelativeTriangle;

  static void getprojPlaneIntersectionPoints(
      std::vector<std::vector<IntersectionCornernessPoint>> &tot_plane_intersections,
      float parallel_threshold, const pcl::PointCloud<pcl::PointNormal> planes);
  static void getIntersectionCornerTriangle(
      std::vector<std::vector<SortRelativeTriangle>> &triangles_in_planes,
      float threshold_cornerness,
      std::vector<std::vector<IntersectionCornernessPoint>> tot_plane_intersections);
  static void findTriangleCorrespondences(
      Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> &score,
      std::vector<std::vector<SortRelativeTriangle>> triangles_in_scan_planes,
      std::vector<std::vector<SortRelativeTriangle>> triangles_in_map_planes);
  static void filterIntersectionPoints(
      std::vector<std::vector<IntersectionCornernessPoint>> &intersection_points);
  static void transformAverage(float (&transformTR)[7], std::vector<int> plane_assignement,
                               const pcl::PointCloud<pcl::PointNormal> scan_planes,
                               const pcl::PointCloud<pcl::PointNormal> map_planes);
};
}  // namespace matching_algorithms
}  // namespace cad_percept