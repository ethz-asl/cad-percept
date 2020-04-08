#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <Eigen/Eigenvalues>

#include <cgal_conversions/eigen_conversions.h>
#include <cgal_definitions/cgal_typedefs.h>

#include "cloud_filtering/cloud_filtering_lib.h"

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

class PlaneMatchLib {
 public:
  static void prrus(float (&transformTR)[7], const pcl::PointCloud<pcl::PointNormal> scan_planes,
                    const pcl::PointCloud<pcl::PointNormal> map_planes,
                    ros::NodeHandle &nh_private);
  static void PlaneDescriptor(float (&transformTR)[7],
                              const pcl::PointCloud<pcl::PointNormal> scan_planes,
                              const pcl::PointCloud<pcl::PointNormal> map_planes);
  static void load_example_sol(float (&transformTR)[7],
                               const pcl::PointCloud<pcl::PointNormal> scan_planes,
                               const pcl::PointCloud<pcl::PointNormal> map_planes,
                               ros::NodeHandle &nh_private);

 private:
  static void getprojPlaneIntersectionPoints(
      std::vector<std::vector<Eigen::Vector3d>> &tot_plane_intersections,
      std::vector<std::vector<std::array<int, 3>>> &used_planes, float parallel_threshold,
      const pcl::PointCloud<pcl::PointNormal> planes);
  static void transform_average(float (&transformTR)[7], std::vector<int> plane_assignement,
                                const pcl::PointCloud<pcl::PointNormal> scan_planes,
                                const pcl::PointCloud<pcl::PointNormal> map_planes);
};
