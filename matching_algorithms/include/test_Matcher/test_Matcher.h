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
  tf::TransformListener tf_listener_;

  // Point Clouds
  bool gotlidar = false;
  bool gotCAD = false;
  bool CAD_ready = false;
  bool lidar_frame_ready = false;
  PointCloud lidar_frame;

  // Param from server
  std::string cad_topic;
  int input_queue_size;
  int map_sampling_density;
  std::string tf_map_frame;

  /**
   * Reference mesh
   */
  cgal::MeshModel::Ptr reference_mesh_;
  PointCloud sample_map;

  // Point cloud variables
  DP ref_dp;

  // Subscribers
  ros::Subscriber cad_sub_;
  ros::Subscriber lidar_sub_;

  // Publisher
  ros::Publisher scan_pub_;
  ros::Publisher map_pub_;

  /**
   * Matcher
   */
  void matcher(float (&transformTR)[6]);

  /**
   * Mesh model callback
   */
  void getCAD(const cgal_msgs::TriangleMeshStamped &cad_mesh_in);

  /**
   * Lidar frame callback
   */
  void getLiDAR(const sensor_msgs::PointCloud2 &cad_mesh_in);

  /**
   * Sample a point cloud from selected triangles of the mesh model
   */
  void sampleFromReferenceFacets(const int density, PointCloud *pointcloud);
};

}  // namespace matching_algorithms
}  // namespace cad_percept

#endif