#ifndef TEST_MATCHER_H_
#define TEST_MATCHER_H_

#include <cgal_conversions/eigen_conversions.h>
#include <cgal_conversions/mesh_conversions.h>
#include <cgal_definitions/mesh_model.h>
#include <cgal_msgs/TriangleMeshStamped.h>
#include <cpt_utils/pc_processing.h>
#include <pcl/point_types.h>
#include <pointmatcher/PointMatcher.h>
#include <pointmatcher_ros/point_cloud.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include "test_Matcher/test_Matcher_parameters.h"

namespace cad_percept {
namespace cpt_matching_algorithms {

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class test_Matcher {
 public:
  test_Matcher(ros::NodeHandle &nh, ros::NodeHandle &nh_private);

 private:
  ros::NodeHandle &nh_, &nh_private_;
  tf::StampedTransform transform;
  Eigen::Matrix3d rotation;
  Eigen::Vector3d translation;
  Eigen::Matrix4d transformation;
  cgal::Transformation ctransformation;
  tf::TransformListener tf_listener_;
  TestMatcherParameters parameters_;

  /**
   * Reference mesh for localization
   */
  cgal::MeshModel::Ptr reference_mesh_;

  // Point cloud variables
  DP ref_dp;

  // Subscribers
  ros::Subscriber cad_sub_;

  // Publisher
  ros::Publisher scan_pub_;
  ros::Publisher map_pub_;

  /**
   * Mesh model callback
   */
  void gotCAD(const cgal_msgs::TriangleMeshStamped &cad_mesh_in);

  /**
   * Sample a point cloud from selected triangles of the mesh model
   */
  void sampleFromReferenceFacets(const int density, PointCloud *pointcloud);
};

}  // namespace cpt_matching_algorithms
}  // namespace cad_percept

#endif