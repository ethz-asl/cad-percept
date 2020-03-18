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

class TestMatcher {
 public:
  TestMatcher(ros::NodeHandle &nh, ros::NodeHandle &nh_private);

 private:
  ros::NodeHandle &nh_, &nh_private_;

  // Cad
  std::string cad_topic;
  cad_percept::cgal::MeshModel::Ptr reference_mesh_;
  float sample_density;

  // given Point Cloud data
  bool use_sim_lidar = false;
  bool lidar_frame_ready = false;
  bool ground_truth_ready = false;
  bool map_ready = false;
  bool ready_for_eval = false;
  PointCloud lidar_frame;
  PointCloud sample_map;

  // Param from server
  int input_queue_size;
  int map_sampling_density;
  std::string tf_map_frame;

  // Evaluation / Ground Truth data
  float transform_TR[6] = {0, 0, 0, 0, 0, 0};  // x y z roll pitch yaw
  geometry_msgs::PointStamped ground_truth;
  float gt_roll;
  float gt_pitch;
  float gt_yaw;

  // Subscribers
  ros::Subscriber map_sub_;
  ros::Subscriber lidar_sub_;
  ros::Subscriber lidar_sim_sub_;
  ros::Subscriber gt_sub_;

  // Publisher
  ros::Publisher scan_pub_;
  ros::Publisher sample_map_pub_;

  /**
   * Sampled map callback
   */
  void getCAD(const cgal_msgs::TriangleMeshStamped &cad_mesh_in);

  /**
   * Lidar frame callback
   */
  void getLiDAR(const sensor_msgs::PointCloud2 &lidar_frame_p2);

  /**
   * Ground truth callback
   */
  void getGroundTruth(const geometry_msgs::PointStamped &gt_in);

  /**
   * Simulated Lidar frame callback
   */
  void getsimLiDAR(const sensor_msgs::PointCloud2 &lidar_frame_p2);

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
  void template_match();
  void go_icp_match();
};

}  // namespace matching_algorithms
}  // namespace cad_percept

#endif