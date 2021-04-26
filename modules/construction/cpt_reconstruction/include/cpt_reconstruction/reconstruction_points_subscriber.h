#ifndef CPT_RECONSTRUCTION_RECONSTRUCTION_POINTS_SUBSCRIBER_H
#define CPT_RECONSTRUCTION_RECONSTRUCTION_POINTS_SUBSCRIBER_H

#include <cpt_reconstruction/reconstruction_preprocess_model.h>

#include <sensor_msgs/PointCloud2.h>
#include "cpt_reconstruction/coordinates.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/point_cloud.h>

namespace cad_percept {
namespace cpt_reconstruction {
class ReconstructionPointsSubscriber {
 public:
  ReconstructionPointsSubscriber() = delete;
  ReconstructionPointsSubscriber(ros::NodeHandle nodeHandle1,
                                 ros::NodeHandle nodeHandle2,
                                 PreprocessModel* model);
  void startReceiving();

 private:
  void messageCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
  ros::NodeHandle nodeHandle1_;
  ros::NodeHandle nodeHandle2_;
  ros::Subscriber subscriber1_;
  ros::Publisher publisher_;
  tf::TransformListener tf_listener_;
  PreprocessModel* model_;
  Eigen::Matrix4d transformation_;
  bool update_transformation_;
  Eigen::Matrix4d transformation_inv_;
  int counter_planes_;
  int counter_cyl_;
  int iteration_counter_;
  pcl::octree::OctreePointCloudDensity<pcl::PointXYZ> octree_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_density_;
};
}  // namespace cpt_reconstruction
}  // namespace cad_percept

#endif  // CPT_RECONSTRUCTION_RECONSTRUCTION_POINTS_SUBSCRIBER_H