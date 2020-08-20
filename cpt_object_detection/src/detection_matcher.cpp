#include "cpt_object_detection/detection_matcher.h"

#include <cgal_msgs/TriangleMeshStamped.h>
#include <cgal_conversions/mesh_conversions.h>

namespace cad_percept {
namespace object_detection {

DetectionMatcher::DetectionMatcher(const ros::NodeHandle& nh,
                                   const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      mesh_model_(nh_private.param<std::string>("off_model", "fail")) {
  subscribeToTopics();
  advertiseTopics();

  LOG(INFO) << "[DetectionMatcher] Object mesh with "
            << mesh_model_.getMesh().size_of_facets()
            << " facets and " << mesh_model_.getMesh().size_of_vertices()
            << " vertices";

  // TODO(gasserl): find appropriate number of points to sample
  constexpr int n_points = 1e3;
  cpt_utils::sample_pc_from_mesh(mesh_model_.getMesh(), n_points, 0.0, &object_pointcloud_);
  LOG(INFO) << "[DetectionMatcher] Converted object mesh with "
            << mesh_model_.getMesh().size_of_facets() << " facets and "
            << mesh_model_.getMesh().size_of_vertices()
            << " vertices to a pointcloud with "
            << object_pointcloud_.size() << " points";

  pcl::toROSMsg(object_pointcloud_, object_pointcloud_msg_);
  object_pointcloud_msg_.header.frame_id = object_frame_id_;

  // init test
  bool visualize_object_on_startup = false;
  nh_private_.param("visualize_object_on_startup",
                    visualize_object_on_startup, visualize_object_on_startup);
  if (visualize_object_on_startup) {
    detection_pointcloud_msg_.header.stamp = ros::Time::now();
    visualizeObjectPointcloud();
    visualizeObjectMesh(detection_frame_id_, object_mesh_init_pub_);
    LOG(INFO) << "[DetectionMatcher] Visualizing object";
  }
}

void DetectionMatcher::subscribeToTopics() {
  std::string pointcloud_topic = "/camera/depth/color/points";
  nh_private_.param("pointcloud_topic", pointcloud_topic, pointcloud_topic);
  int queue_size = 1;
  nh_private_.param("queue_size", queue_size, queue_size);
  detection_pointcloud_sub_ =
      nh_.subscribe(pointcloud_topic, queue_size,
                    &DetectionMatcher::pointcloudCallback, this);
  LOG(INFO) << "[DetectionMatcher] Subscribed to pointcloud topic ["
            << detection_pointcloud_sub_.getTopic() << "]";
}

void DetectionMatcher::advertiseTopics() {
  object_pointcloud_pub_ =
      nh_private_.advertise<sensor_msgs::PointCloud2>("object_pcl", 1, true);
  LOG(INFO) << "[DetectionMatcher] Publishing object poincloud to topic ["
            << object_pointcloud_pub_.getTopic() << "]";
  object_mesh_pub_ =
      nh_private_.advertise<cgal_msgs::TriangleMeshStamped>("object_mesh", 1, true);
  LOG(INFO) << "[DetectionMatcher] Publishing object mesh to topic ["
            << object_mesh_pub_.getTopic() << "]";
}

void DetectionMatcher::pointcloudCallback(const sensor_msgs::PointCloud2 &cloud_msg_in) {
  detection_frame_id_ = cloud_msg_in.header.frame_id;
  detection_pointcloud_msg_ = cloud_msg_in;
  pcl::fromROSMsg(detection_pointcloud_msg_, detection_pointcloud_);
}

void DetectionMatcher::visualizeObjectMesh(
    const std::string& frame_id, const ros::Publisher& publisher) const {
  cgal_msgs::TriangleMeshStamped p_msg;

  // triangle mesh to prob. msg
  cgal_msgs::TriangleMesh t_msg;
  cgal::Polyhedron mesh = mesh_model_.getMesh();
  cgal::triangleMeshToMsg(mesh, &t_msg);
  p_msg.mesh = t_msg;

  p_msg.header.frame_id = frame_id;
  p_msg.header.stamp = detection_pointcloud_msg_.header.stamp;
  p_msg.header.seq = 0;
  publisher.publish(p_msg);
}

void DetectionMatcher::visualizeObjectPointcloud() {
  object_pointcloud_pub_.publish(object_pointcloud_msg_);
}

}
}