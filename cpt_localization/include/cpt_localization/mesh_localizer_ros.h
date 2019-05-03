#ifndef CPT_LOCALIZATION_MESH_LOCALIZER_ROS_H
#define CPT_LOCALIZATION_MESH_LOCALIZER_ROS_H

#include <kindr/minimal/quat-transformation-gtsam.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <shape_msgs/Mesh.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include "mesh_localizer.h"

namespace cad_percept {
namespace localization {

typedef kindr::minimal::QuatTransformationTemplate<double> SE3;
typedef SE3::Rotation SO3;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class MeshLocalizerRos {
 public:
  MeshLocalizerRos(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
  ~MeshLocalizerRos();

  // Record the current pose of the robot.
  void recordPose(const geometry_msgs::TransformStamped &transform_msg);

  // Associate point-cloud with architect model.
  void associatePointCloud(const PointCloud &pc_msg);

  // Service call to transform the architect model.
  bool transformModelCb(std_srvs::Empty::Request &request,
                        std_srvs::Empty::Response &response);

  // Publishing of architect model as point cloud.
  void publishArchitectModel() const;

 private:
  MeshLocalizer mesh_localizer_;
  ros::NodeHandle &nh_, nh_private_;
  ros::Publisher good_matches_pub_, bad_matches_pub_, model_pub_, arch_pub_,
  pose_pub_;
  ros::Subscriber pointcloud_sub_, icp_sub_;
  visualization_msgs::Marker model_;
  ros::ServiceServer transformSrv_;
  tf::TransformListener tf_listener_;
  std::string map_frame_, cad_frame_;
  SE3 current_pose_;
  double distance_threshold_;
};
}
}

#endif