// PlaneExtractionLib.h

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

class PlaneExtractionLib {
 public:
  static void rht_plane_extraction(std::vector<pcl::PointCloud<pcl::PointXYZ>> &extracted_planes,
                                   std::vector<std::vector<double>> &plane_coefficients,
                                   const pcl::PointCloud<pcl::PointXYZ> lidar_scan,
                                   ros::Publisher &plane_pub, std::string tf_map_frame,
                                   ros::NodeHandle &nh_private);
  static void iter_rht_plane_extraction(
      std::vector<pcl::PointCloud<pcl::PointXYZ>> &extracted_planes,
      std::vector<std::vector<double>> &plane_coefficients,
      const pcl::PointCloud<pcl::PointXYZ> lidar_scan, ros::Publisher &plane_pub,
      std::string tf_map_frame, ros::NodeHandle &nh_private);
  static void pcl_plane_extraction(std::vector<pcl::PointCloud<pcl::PointXYZ>> &extracted_planes,
                                   std::vector<std::vector<double>> &plane_coefficients,
                                   const pcl::PointCloud<pcl::PointXYZ> lidar_scan,
                                   ros::Publisher &plane_pub, std::string tf_map_frame,
                                   ros::NodeHandle &nh_private);
  static void visualize_plane(std::vector<pcl::PointCloud<pcl::PointXYZ>> &extracted_planes,
                              ros::Publisher &plane_pub, std::string tf_map_frame);

 private:
  class HoughAccumulator;
  class ArrayAccumulator;
  class BallAccumulator;

  static void rht_vote(int max_iteration, double tol_distance_between_points,
                       double min_area_spanned, const pcl::PointCloud<pcl::PointXYZ> lidar_scan,
                       PlaneExtractionLib::HoughAccumulator *accumulator);
  static std::vector<std::vector<int>> rht_eval(
      int num_main_planes, int min_vote_threshold, int k_of_maxima_suppression,
      std::vector<std::vector<double>> &plane_coefficients,
      std::vector<pcl::PointCloud<pcl::PointXYZ>> &extracted_planes,
      const pcl::PointCloud<pcl::PointXYZ> lidar_scan,
      PlaneExtractionLib::HoughAccumulator *accumulator);
};
