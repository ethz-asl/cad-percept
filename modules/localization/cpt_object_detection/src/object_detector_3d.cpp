#include "cpt_object_detection/object_detector_3d.h"

#include <cgal_conversions/mesh_conversions.h>
#include <cgal_msgs/TriangleMeshStamped.h>
#include <cpt_utils/pc_processing.h>
#include <minkindr_conversions/kindr_msg.h>
#include <tf/transform_broadcaster.h>

namespace cad_percept {
namespace object_detection {

ObjectDetector3D::ObjectDetector3D(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      pointcloud_topic_("/camera/depth/color/points"),
      object_frame_id_("object_detection_mesh") {
  getParamsFromRos();
  subscribeToTopics();
  advertiseTopics();

  const std::string& off_file = nh_private_.param<std::string>("off_model", "fail");
  if (!cgal::MeshModel::create(off_file, &mesh_model_)) {
    LOG(FATAL) << "Could not get mesh model from off file at " << off_file << "!";
  }
  LOG(INFO) << "Object mesh with " << mesh_model_->getMesh().size_of_facets() << " facets and "
            << mesh_model_->getMesh().size_of_vertices() << " vertices";

  processMesh();
}

void ObjectDetector3D::getParamsFromRos() {
  nh_private_.param("pointcloud_topic", pointcloud_topic_, pointcloud_topic_);
  nh_private_.param("object_frame_id", object_frame_id_, object_frame_id_);
  nh_private_.param("icp_config_file", icp_config_file_, icp_config_file_);
}

void ObjectDetector3D::subscribeToTopics() {
  int queue_size = 1;
  nh_private_.param("queue_size", queue_size, queue_size);
  detection_pointcloud_sub_ = nh_.subscribe(pointcloud_topic_, queue_size,
                                            &ObjectDetector3D::objectDetectionCallback, this);
  LOG(INFO) << "Subscribed to pointcloud topic [" << detection_pointcloud_sub_.getTopic() << "]";
}

void ObjectDetector3D::advertiseTopics() {
  object_pointcloud_pub_ = nh_private_.advertise<sensor_msgs::PointCloud2>("object_pcl", 1, true);
  LOG(INFO) << "Publishing object poincloud to topic [" << object_pointcloud_pub_.getTopic() << "]";
  object_mesh_pub_ = nh_private_.advertise<cgal_msgs::TriangleMeshStamped>("object_mesh", 1, true);
  LOG(INFO) << "Publishing object mesh to topic [" << object_mesh_pub_.getTopic() << "]";
  object_mesh_init_pub_ =
      nh_private_.advertise<cgal_msgs::TriangleMeshStamped>("object_mesh_init", 1, true);
  LOG(INFO) << "Publishing init object mesh to topic [" << object_mesh_init_pub_.getTopic() << "]";
}

void ObjectDetector3D::processMesh() {
  // Sample pointcloud from object mesh
  int num_points_object_pointcloud = 1e3;
  nh_private_.param("num_points_object_pointcloud", num_points_object_pointcloud,
                    num_points_object_pointcloud);
  cpt_utils::sample_pc_from_mesh(mesh_model_->getMesh(), num_points_object_pointcloud, 0.0,
                                 &object_pointcloud_);
  LOG(INFO) << "Converted object mesh with " << mesh_model_->getMesh().size_of_facets()
            << " facets and " << mesh_model_->getMesh().size_of_vertices()
            << " vertices to a pointcloud with " << object_pointcloud_.size() << " points";

  // Visualize object
  bool visualize_object_on_startup = false;
  nh_private_.param("visualize_object_on_startup", visualize_object_on_startup,
                    visualize_object_on_startup);
  if (visualize_object_on_startup) {
    visualizePointcloud(object_pointcloud_, ros::Time::now(), detection_frame_id_,
                        object_pointcloud_pub_);
    visualizeMesh(mesh_model_, ros::Time::now(), detection_frame_id_, object_mesh_init_pub_);
    LOG(INFO) << "Visualizing object";
  }
}

void ObjectDetector3D::objectDetectionCallback(const sensor_msgs::PointCloud2& cloud_msg_in) {
  detection_frame_id_ = cloud_msg_in.header.frame_id;
  detection_stamp_ = cloud_msg_in.header.stamp;
  pcl::fromROSMsg(cloud_msg_in, detection_pointcloud_);

  processDetectionUsingPcaAndIcp();
}

void ObjectDetector3D::processDetectionUsingPcaAndIcp() {
  Transformation T_object_detection_init;
  Transformation T_object_detection = alignDetectionUsingPcaAndIcp(
      object_pointcloud_, detection_pointcloud_, icp_config_file_, &T_object_detection_init);

  // Publish transformations to TF
  publishTransformation(T_object_detection_init.inverse(), detection_stamp_, detection_frame_id_,
                        object_frame_id_ + "_init");
  publishTransformation(T_object_detection.inverse(), detection_stamp_, detection_frame_id_,
                        object_frame_id_);

  // Visualize object
  visualizeMesh(mesh_model_, detection_stamp_, object_frame_id_ + "_init", object_mesh_init_pub_);
  visualizeMesh(mesh_model_, detection_stamp_, object_frame_id_, object_mesh_pub_);
  visualizePointcloud(object_pointcloud_, detection_stamp_, detection_frame_id_,
                      object_pointcloud_pub_);
}

void ObjectDetector3D::publishTransformation(const Transformation& transform,
                                             const ros::Time& stamp,
                                             const std::string& parent_frame_id,
                                             const std::string& child_frame_id) {
  static tf2_ros::TransformBroadcaster tf_broadcaster;
  geometry_msgs::Transform transform_msg;
  tf::transformKindrToMsg(transform.cast<double>(), &transform_msg);
  geometry_msgs::TransformStamped stamped_transform_msg;
  stamped_transform_msg.header.stamp = stamp;
  stamped_transform_msg.header.frame_id = parent_frame_id;
  stamped_transform_msg.child_frame_id = child_frame_id;
  stamped_transform_msg.transform = transform_msg;
  tf_broadcaster.sendTransform(stamped_transform_msg);
}

void ObjectDetector3D::visualizeMesh(const cgal::MeshModel::Ptr& mesh_model,
                                     const ros::Time& timestamp, const std::string& frame_id,
                                     const ros::Publisher& publisher) {
  cgal_msgs::TriangleMeshStamped p_msg;

  // triangle mesh to prob. msg
  cgal_msgs::TriangleMesh t_msg;
  cgal::Polyhedron mesh = mesh_model->getMesh();
  cgal::triangleMeshToMsg(mesh, &t_msg);
  p_msg.mesh = t_msg;

  p_msg.header.frame_id = frame_id;
  p_msg.header.stamp = timestamp;
  p_msg.header.seq = 0;
  publisher.publish(p_msg);
}

void ObjectDetector3D::visualizePointcloud(const pcl::PointCloud<pcl::PointXYZ>& pointcloud,
                                           const ros::Time& timestamp, const std::string& frame_id,
                                           const ros::Publisher& publisher) {
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(pointcloud, msg);
  msg.header.stamp = timestamp;
  msg.header.frame_id = frame_id;
  publisher.publish(msg);
}

}  // namespace object_detection
}  // namespace cad_percept