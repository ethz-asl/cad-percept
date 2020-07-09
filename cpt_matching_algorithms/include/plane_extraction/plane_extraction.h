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

#include <CGAL/pca_estimate_normals.h>
#include <CGAL/regularize_planes.h>

#include "plane_extraction/rht_accumulator.h"

namespace cad_percept {
namespace matching_algorithms {

class PlaneExtractor {
 public:
  // Loads Config for RHT from Server
  struct rhtConfig {
    int accumulator_choice;
    double rho_resolution;
    double theta_resolution;
    double psi_resolution;
    int min_vote_threshold;
    int k_of_maxima_suppression;

    int max_iteration;
    double tol_distance_between_points;
    double min_area_spanned;
    int num_main_planes;
  };
  static rhtConfig loadRhtConfigFromServer();
  // Returns normal and inliers of planes in point cloud lidar_scan using RHT without removing
  // detected planes
  static void rhtPlaneExtraction(std::vector<pcl::PointCloud<pcl::PointXYZ>> &extracted_planes_out,
                                 std::vector<Eigen::Vector3d> &plane_normals_out,
                                 const pcl::PointCloud<pcl::PointXYZ> &lidar_scan,
                                 const rhtConfig &config);

  // Loads Config for iterative RHT from Server
  struct iterRhtConfig {
    int accumulator_choice;
    double rho_resolution;
    double theta_resolution;
    double psi_resolution;
    int k_of_maxima_suppression;
    int min_vote_thresh;

    int num_rht_iter;
    int num_vote_iter;
    double dist_thresh;
    double min_area;
    int num_plane_per_iter;
    int min_num_inlier;
  };
  static iterRhtConfig loadIterRhtConfigFromServer();
  // Returns normal and inliers of planes in point cloud lidar_scan using RHT with removing
  // detected planes
  static void iterRhtPlaneExtraction(
      std::vector<pcl::PointCloud<pcl::PointXYZ>> &extracted_planes_out,
      std::vector<Eigen::Vector3d> &plane_normals_out,
      const pcl::PointCloud<pcl::PointXYZ> &lidar_scan, const iterRhtConfig &config);

  // Loads Config for PCL RANSAC from Server
  struct pclRansacConfig {
    double dist_thresh;
    int max_num_plane;
    int min_num_inlier;
  };
  static pclRansacConfig loadPclRansacConfigFromServer();
  // Returns normal and inliers of planes in point cloud lidar_scan using pcl RANSAC plane detection
  static void pclPlaneExtraction(std::vector<pcl::PointCloud<pcl::PointXYZ>> &extracted_planes_out,
                                 std::vector<Eigen::Vector3d> &plane_normals_out,
                                 const pcl::PointCloud<pcl::PointXYZ> &lidar_scan,
                                 const pclRansacConfig &config);

  // Loads Config for CGAL RG from Server
  struct cgalRgConfig {
    int min_num_inliers;
    float max_dist_to_plane;
    float max_dist_betw_point;
    float diff_normal_tresh;

    bool regul_activated;
    bool regul_paral;
    bool regul_orth;
    bool regul_coplanar;
    float regul_paral_orth_thresh;
    float regul_coplanar_thresh;
  };
  static cgalRgConfig loadCgalRgConfigFromServer();
  // Returns normal and inliers of planes in point cloud lidar_scan using CGAL Region Growing
  static void cgalRegionGrowing(std::vector<pcl::PointCloud<pcl::PointXYZ>> &extracted_planes_out,
                                std::vector<Eigen::Vector3d> &plane_normals_out,
                                const pcl::PointCloud<pcl::PointXYZ> &lidar_scan,
                                const cgalRgConfig &config);

  // Publishes RGB point clouds representing segmentation of detected planes
  static void visualizePlane(const std::vector<pcl::PointCloud<pcl::PointXYZ>> &extracted_planes,
                             ros::Publisher &plane_pub, const std::string &tf_map_frame);

 private:
  // Runs max_iteration voting iterations
  static void rhtVote(int max_iteration, double tol_distance_between_points,
                      double min_area_spanned, const pcl::PointCloud<pcl::PointXYZ> &lidar_scan,
                      std::vector<int> removed_pc_to_pc, HoughAccumulator *accumulator);

  // Returns detected planes considering voting in accumulator and applies Non-maxima Suppression
  static std::vector<std::vector<int>> rhtEval(
      int num_main_planes, int min_vote_threshold, int k_of_maxima_suppression,
      std::vector<Eigen::Vector3d> &plane_normals_out,
      std::vector<pcl::PointCloud<pcl::PointXYZ>> &extracted_planes_out,
      const pcl::PointCloud<pcl::PointXYZ> &lidar_scan, HoughAccumulator *accumulator);
};

}  // namespace matching_algorithms
}  // namespace cad_percept

#endif