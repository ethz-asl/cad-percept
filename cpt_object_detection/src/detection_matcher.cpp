#include "cpt_object_detection/detection_matcher.h"

#include <geometry_msgs/TransformStamped.h>
#include <cgal_msgs/TriangleMeshStamped.h>
#include <cgal_conversions/mesh_conversions.h>
#include <tf/transform_broadcaster.h>
#include <pcl/common/pca.h>
#include <cpt_utils/pc_processing.h>

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
  object_mesh_init_pub_ =
      nh_private_.advertise<cgal_msgs::TriangleMeshStamped>("object_mesh_init", 1, true);
  LOG(INFO) << "[DetectionMatcher] Publishing init object mesh to topic ["
            << object_mesh_init_pub_.getTopic() << "]";
}

void DetectionMatcher::pointcloudCallback(const sensor_msgs::PointCloud2 &cloud_msg_in) {
  detection_frame_id_ = cloud_msg_in.header.frame_id;
  detection_pointcloud_msg_ = cloud_msg_in;
  pcl::fromROSMsg(detection_pointcloud_msg_, detection_pointcloud_);

  processPointcloud();
}

void DetectionMatcher::processPointcloud() {
  ros::WallTime time_start = ros::WallTime::now();

  // get initial guess
  kindr::minimal::QuatTransformationTemplate<float> T_object_detection_init;
  if(!findInitialGuess(&T_object_detection_init)) {
    LOG(WARNING) << "Initialization of ICP from detection pointcloud to "
                    "object pointcloud failed!";
    T_object_detection_init.setIdentity();
  }
  publishTransformation(T_object_detection_init.inverse(),
                        detection_pointcloud_msg_.header.stamp,
                        detection_frame_id_, object_frame_id_ + "_init");
  visualizeObjectMesh(object_frame_id_ + "_init", object_mesh_init_pub_);
  LOG(INFO) << "Time: " << (ros::WallTime::now() - time_start).toSec();
}

bool DetectionMatcher::findInitialGuess(
    kindr::minimal::QuatTransformationTemplate<float>* T_object_detection_init) {
  ros::WallTime time_start = ros::WallTime::now();

  if (detection_pointcloud_.size() < 3) {
    LOG(WARNING) << "Detection PCA not possible! Too few points: "
                 << detection_pointcloud_.size();
    return false;
  }
  if (object_pointcloud_.size() >= 3) {
    LOG(WARNING) << "Object PCA not possible! Too few points: "
                 << object_pointcloud_.size();
    return false;
  }

  // Get data from detection pointcloud
  pcl::PCA<pcl::PointXYZ> detection_pca;
  pcl::PointCloud<pcl::PointXYZ>::Ptr detection_pointcloud_ptr =
      detection_pointcloud_.makeShared();
  detection_pca.setInputCloud(detection_pointcloud_ptr);
  Eigen::Vector4f detection_centroid = detection_pca.getMean();
  Eigen::Matrix3f detection_vectors = detection_pca.getEigenVectors();

  // Get data from object pointcloud
  pcl::PCA<pcl::PointXYZ> object_pca;
  pcl::PointCloud<pcl::PointXYZ>::Ptr object_pointcloud_ptr =
      object_pointcloud_.makeShared();
  object_pca.setInputCloud(object_pointcloud_ptr);
  Eigen::Vector4f object_centroid = object_pca.getMean();
  Eigen::Matrix3f object_vectors = object_pca.getEigenVectors();

  // Translation from mean of pointclouds det_r_obj_det
  kindr::minimal::PositionTemplate<float> translation(
      detection_centroid.head(3) - object_centroid.head(3));

  // Get coordinate system to be right
  if (detection_vectors.determinant() < 0) {
    detection_vectors.col(2) = -detection_vectors.col(2);
  }
  if (object_vectors.determinant() < 0) {
    object_vectors.col(2) = -object_vectors.col(2);
  }
  // Get rotation between detection and object pointcloud R_obj_det
  Eigen::Matrix3f rotation_matrix =
      detection_vectors * object_vectors.transpose();

  kindr::minimal::RotationQuaternionTemplate<float> rotation;
  rotation.setIdentity();
  if (kindr::minimal::RotationQuaternionTemplate<float>::isValidRotationMatrix(rotation_matrix)) {
    rotation = kindr::minimal::RotationQuaternionTemplate<float>(rotation_matrix);
  } else {
    LOG(WARNING) << "Rotation matrix is not valid!";
    LOG(INFO) << "determinant: " << rotation_matrix.determinant();
    LOG(INFO) << "R*R^T:\n" << rotation_matrix * rotation_matrix.transpose();
    return false;
  }

  *T_object_detection_init =
      kindr::minimal::QuatTransformationTemplate<float>(rotation,
                                                        translation).inverse();
  LOG(INFO) << "Time initial guess: " << (ros::WallTime::now() - time_start).toSec();
  return true;
}

void DetectionMatcher::publishTransformation(
    const kindr::minimal::QuatTransformationTemplate<float>& transform,
    const ros::Time& stamp, const std::string& parent_frame_id,
    const std::string& child_frame_id) const {
  static tf::TransformBroadcaster tf_broadcaster;
  tf::Transform tf_transform;
  tf_transform.setOrigin(tf::Vector3(transform.getPosition().x(),
                                     transform.getPosition().y(),
                                     transform.getPosition().z()));
  tf_transform.setRotation(tf::Quaternion(transform.getRotation().x(),
                                          transform.getRotation().y(),
                                          transform.getRotation().z(),
                                          transform.getRotation().w()));
  const tf::StampedTransform tf_transform_stamped(
      tf_transform, stamp, parent_frame_id, child_frame_id);
  tf_broadcaster.sendTransform(tf_transform_stamped);
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