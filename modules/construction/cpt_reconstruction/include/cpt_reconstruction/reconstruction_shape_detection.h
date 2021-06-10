#ifndef CPT_RECONSTRUCTION_RECONSTRUCTION_POINTS_SUBSCRIBER_H
#define CPT_RECONSTRUCTION_RECONSTRUCTION_POINTS_SUBSCRIBER_H

#include <cpt_reconstruction/reconstruction_model.h>

#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include "cpt_reconstruction/coordinates.h"
#include "cpt_reconstruction/shape.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

namespace cad_percept {
namespace cpt_reconstruction {
class ShapeDetection {
 public:
  ShapeDetection() = delete;
  ShapeDetection(ros::NodeHandle nodeHandle1, ros::NodeHandle nodeHandle2,
                 Model* model);
  void startReceiving();

 private:
  // Parameters
  int SENSOR_TYPE_;
  std::vector<double> TRANSFORMATION_VEC_;
  std::vector<double> STATIONARY_POSITION_VEC_;
  Eigen::Matrix4d TRANSFORMATION_;
  double MODEL_TOLERANCE_;
  int OUTLIER_COUNT_;
  bool USE_BUFFER_;
  int CLEAR_BUFFER_AFTER_ITERATION_;
  std::string ALL_POINTS_PATH_;
  std::string OUTLIER_POINTS_PATH_;

  void messageCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
  ros::NodeHandle nodeHandle1_;
  ros::NodeHandle nodeHandle2_;
  ros::Subscriber subscriber1_;
  ros::Publisher publisher_;
  tf::TransformListener tf_listener_;
  Model* model_;
  Eigen::Matrix4d transformation_;
  Eigen::Vector3d stationary_position_;
  bool update_transformation_;
  Eigen::Matrix4d transformation_inv_;
  int counter_planes_;
  int counter_cyl_;
  int iteration_counter_;
};
}  // namespace cpt_reconstruction
}  // namespace cad_percept

#endif  // CPT_RECONSTRUCTION_RECONSTRUCTION_POINTS_SUBSCRIBER_H