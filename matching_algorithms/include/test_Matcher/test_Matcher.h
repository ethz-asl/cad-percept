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

namespace cad_percept {
namespace matching_algorithms {

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class test_Matcher {
 public:
  test_Matcher(ros::NodeHandle &nh, ros::NodeHandle &nh_private);

 private:
  ros::NodeHandle &nh_, &nh_private_;

  // given Point Cloud data
  bool usesimlidar = false;
  bool lidar_frame_ready = false;
  bool ground_truth_ready = false;
  bool map_ready = false;
  PointCloud lidar_frame;
  PointCloud sample_map;

  // Param from server
  std::string cad_topic;
  int input_queue_size;
  int map_sampling_density;
  std::string tf_map_frame;

  // Evaluation / Ground Truth data
  geometry_msgs::PointStamped ground_truth;
  float gtroll;
  float gtpitch;
  float gtyaw;

  // ROS
  DP ref_dp;

  // Subscribers
  ros::Subscriber sample_map_sub_;
  ros::Subscriber lidar_sub_;
  ros::Subscriber lidar_sim_sub_;
  ros::Subscriber gt_sub_;

  // Publisher
  ros::Publisher scan_pub_;

  /**
   * Sampled map callback
   */
  void getsampleCAD(const sensor_msgs::PointCloud2 &cad_map);

  /**
   * Lidar frame callback
   */
  void getLiDAR(const sensor_msgs::PointCloud2 &lidarframe);

  /**
   * Ground truth callback
   */
  void getGroundTruth(const geometry_msgs::PointStamped &gt_in);

  /**
   * Simulated Lidar frame callback
   */
  void getsimLiDAR(const sensor_msgs::PointCloud2 &lidarframe);

  /**
   * Calcualte RMSE (root mean square error) of two point clouds
   */
  void getError(PointCloud p1, PointCloud p2);

  /**
   * Declare matchers
   */
  void template_match(float (&transformTR)[6]);
};

}  // namespace matching_algorithms
}  // namespace cad_percept

#endif