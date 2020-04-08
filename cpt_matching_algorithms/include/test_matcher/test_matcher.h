#ifndef TEST_MATCHER_H_
#define TEST_MATCHER_H_

#include <cgal_conversions/eigen_conversions.h>
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

#include "cloud_filter/cloud_filter.h"
#include "plane_extraction/plane_extraction.h"

namespace cad_percept {
namespace matching_algorithms {

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class TestMatcher {
 public:
  TestMatcher(ros::NodeHandle &nh, ros::NodeHandle &nh_private);

 private:
  ros::NodeHandle &nh_, &nh_private_;

  // Cad
  std::string cad_topic_;
  cad_percept::cgal::MeshModel::Ptr reference_mesh_;
  float sample_density_;

  // given Point Cloud data
  bool use_sim_lidar_ = false;
  bool lidar_scan_ready_ = false;
  bool ground_truth_ready_ = false;
  bool map_ready_ = false;
  bool ready_for_eval_ = false;
  PointCloud lidar_scan_;
  PointCloud sample_map_;
  pcl::PointCloud<pcl::PointXYZI> static_structure_cloud_;

  // Param from server
  int input_queue_size_;
  int map_sampling_density_;
  std::string tf_map_frame_;

  // Evaluation / Ground Truth data
  float transform_TR_[7] = {0, 0, 0, 0, 0, 0, 0};  // x y z qw qx qy qz
  geometry_msgs::PointStamped ground_truth_;
  std::vector<float> gt_quat_;

  // Subscribers
  std::string lidar_topic_;
  std::string sim_lidar_topic_;
  std::string ground_truth_topic_;
  ros::Subscriber map_sub_;
  ros::Subscriber lidar_sub_;
  ros::Subscriber lidar_sim_sub_;
  ros::Subscriber gt_sub_;

  // Publisher
  ros::Publisher scan_pub_;
  ros::Publisher sample_map_pub_;
  ros::Publisher plane_pub_;

  /**
   * Sampled map callback
   */
  void getCAD(const cgal_msgs::TriangleMeshStamped &cad_mesh_in);

  /**
   * Lidar frame callback
   */
  void getLidar(const sensor_msgs::PointCloud2 &lidar_scan_p2);

  /**
   * Ground truth callback
   */
  void getGroundTruth(const geometry_msgs::PointStamped &gt_in);

  /**
   * Simulated Lidar frame callback
   */
  void getSimLidar(const sensor_msgs::PointCloud2 &lidar_scan_p2);

  /**
   * Match function
   */
  void match();

  /**
   * Evaluation function
   */
  void evaluate();

  /**
   * Calcualte RMSE (root mean square error) of two point clouds
   */
  void getError(PointCloud p1, PointCloud p2);

  /**
   * Declare matchers
   */
  void templateMatch();
  void goicpMatch();
};

}  // namespace matching_algorithms
}  // namespace cad_percept

#endif