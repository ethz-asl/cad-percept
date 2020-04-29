#ifndef PLANE_EXTRACTION_H_
#define PLANE_EXTRACTION_H_

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

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/common/geometry.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <CGAL/pca_estimate_normals.h>
#include <CGAL/regularize_planes.h>

#include "plane_extraction/rht_accumulator.h"

namespace cad_percept {
namespace matching_algorithms {

class PlaneExtractor {
 public:
  // Returns normal and inliers of planes in point cloud lidar_scan using RHT without removing
  // detected planes
  static void rhtPlaneExtraction(std::vector<pcl::PointCloud<pcl::PointXYZ>> &extracted_planes,
                                 std::vector<Eigen::Vector3d> &plane_normals,
                                 pcl::PointCloud<pcl::PointXYZ> lidar_scan,
                                 std::string tf_map_frame, ros::Publisher &plane_pub);
  // Returns normal and inliers of planes in point cloud lidar_scan using RHT with removing
  // detected planes
  static void iterRhtPlaneExtraction(std::vector<pcl::PointCloud<pcl::PointXYZ>> &extracted_planes,
                                     std::vector<Eigen::Vector3d> &plane_normals,
                                     pcl::PointCloud<pcl::PointXYZ> lidar_scan,
                                     std::string tf_map_frame, ros::Publisher &plane_pub);
  // Returns normal and inliers of planes in point cloud lidar_scan using pcl RANSAC plane detection
  static void pclPlaneExtraction(std::vector<pcl::PointCloud<pcl::PointXYZ>> &extracted_planes,
                                 std::vector<Eigen::Vector3d> &plane_normals,
                                 pcl::PointCloud<pcl::PointXYZ> lidar_scan,
                                 std::string tf_map_frame, ros::Publisher &plane_pub);
  // Returns normal and inliers of planes in point cloud lidar_scan using CGAL Region Growing
  static void cgalRegionGrowing(std::vector<pcl::PointCloud<pcl::PointXYZ>> &extracted_planes,
                                std::vector<Eigen::Vector3d> &plane_normals,
                                pcl::PointCloud<pcl::PointXYZ> lidar_scan, std::string tf_map_frame,
                                ros::Publisher &plane_pub);
  // Publishes RGB point clouds representing segmentation of detected planes
  static void visualizePlane(std::vector<pcl::PointCloud<pcl::PointXYZ>> &extracted_planes,
                             ros::Publisher &plane_pub, std::string tf_map_frame);

 private:
  // Runs max_iteration voting iterations
  static void rhtVote(int max_iteration, double tol_distance_between_points,
                      double min_area_spanned, pcl::PointCloud<pcl::PointXYZ> lidar_scan,
                      HoughAccumulator *accumulator);
  // Returns detected planes considering voting in accumulator and applies Non-maxima Suppression
  static std::vector<std::vector<int>> rhtEval(
      int num_main_planes, int min_vote_threshold, int k_of_maxima_suppression,
      std::vector<Eigen::Vector3d> &plane_normals,
      std::vector<pcl::PointCloud<pcl::PointXYZ>> &extracted_planes,
      const pcl::PointCloud<pcl::PointXYZ> lidar_scan, HoughAccumulator *accumulator);
};

}  // namespace matching_algorithms
}  // namespace cad_percept

#endif