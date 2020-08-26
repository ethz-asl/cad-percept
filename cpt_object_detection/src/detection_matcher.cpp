#include "cpt_object_detection/detection_matcher.h"

#include <geometry_msgs/TransformStamped.h>
#include <cgal_msgs/TriangleMeshStamped.h>
#include <cgal_conversions/mesh_conversions.h>
#include <tf/transform_broadcaster.h>
#include <pointmatcher_ros/point_cloud.h>
#include <pcl/common/pca.h>
#include <cpt_utils/pc_processing.h>

namespace cad_percept {
namespace object_detection {

DetectionMatcher::DetectionMatcher(const ros::NodeHandle& nh,
                                   const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      mesh_model_(nh_private.param<std::string>("off_model", "fail")),
      visualize_object_on_startup_(false),
      object_frame_id_("object_detection_mesh"),
      num_points_object_pointcloud_(500),
      num_points_icp_(500) {
  getParamsFromRos();
  subscribeToTopics();
  advertiseTopics();

  LOG(INFO) << "[DetectionMatcher] Object mesh with "
            << mesh_model_.getMesh().size_of_facets()
            << " facets and " << mesh_model_.getMesh().size_of_vertices()
            << " vertices";

  processObject();
}

void DetectionMatcher::getParamsFromRos() {
  nh_private_.param("visualize_object_on_startup",
                    visualize_object_on_startup_,
                    visualize_object_on_startup_);
  nh_private_.param("object_frame_id",
                    object_frame_id_, object_frame_id_);
  nh_private_.param("num_points_object_pointcloud",
                    num_points_object_pointcloud_,
                    num_points_object_pointcloud_);
  nh_private_.param("num_points_icp", num_points_icp_, num_points_icp_);
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

void DetectionMatcher::processObject() {
  // TODO(gasserl): find appropriate number of points to sample
  cpt_utils::sample_pc_from_mesh(mesh_model_.getMesh(),
                                 num_points_object_pointcloud_, 0.0,
                                 &object_pointcloud_);
  LOG(INFO) << "[DetectionMatcher] Converted object mesh with "
            << mesh_model_.getMesh().size_of_facets() << " facets and "
            << mesh_model_.getMesh().size_of_vertices()
            << " vertices to a pointcloud with "
            << object_pointcloud_.size() << " points";

  pcl::toROSMsg(object_pointcloud_, object_pointcloud_msg_);
  object_pointcloud_msg_.header.frame_id = object_frame_id_;

  // init test
  if (visualize_object_on_startup_) {
    detection_pointcloud_msg_.header.stamp = ros::Time::now();
    object_pointcloud_msg_.header.frame_id = detection_frame_id_;
    visualizeObjectPointcloud();
    visualizeObjectMesh(detection_frame_id_, object_mesh_init_pub_);
    LOG(INFO) << "[DetectionMatcher] Visualizing object";
    object_pointcloud_msg_.header.frame_id = object_frame_id_;
  }
}

void DetectionMatcher::pointcloudCallback(const sensor_msgs::PointCloud2 &cloud_msg_in) {
  detection_frame_id_ = cloud_msg_in.header.frame_id;
  detection_pointcloud_msg_ = cloud_msg_in;
  pcl::fromROSMsg(detection_pointcloud_msg_, detection_pointcloud_);

  processPointcloudUsingPcaAndIcp();
}

void DetectionMatcher::processPointcloudUsingPcaAndIcp() {
  ros::WallTime time_start = ros::WallTime::now();

  // get initial guess
  kindr::minimal::QuatTransformationTemplate<float> T_object_detection_init;
  if(!findInitialGuessUsingIcp(&T_object_detection_init)) {
    LOG(WARNING) << "Initialization of ICP from detection pointcloud to "
                    "object pointcloud failed!";
    T_object_detection_init.setIdentity();
  }
  publishTransformation(T_object_detection_init.inverse(),
                        detection_pointcloud_msg_.header.stamp,
                        detection_frame_id_, object_frame_id_ + "_init");
  visualizeObjectMesh(object_frame_id_ + "_init", object_mesh_init_pub_);

  // ICP with initial guess
  kindr::minimal::QuatTransformationTemplate<float> T_object_detection;
  if(!performICP(T_object_detection_init, &T_object_detection)) {
    LOG(WARNING) << "ICP from detection pointcloud to "
                    "object pointcloud failed!";
    T_object_detection.setIdentity();
  }

  // Publish results
  publishTransformation(T_object_detection.inverse(),
                        detection_pointcloud_msg_.header.stamp,
                        detection_frame_id_, object_frame_id_);
  object_pointcloud_msg_.header.stamp = detection_pointcloud_msg_.header.stamp;
  visualizeObjectPointcloud();
  visualizeObjectMesh(object_frame_id_, object_mesh_pub_);
  LOG(INFO) << "Time: " << (ros::WallTime::now() - time_start).toSec();
}

bool DetectionMatcher::findInitialGuessUsingIcp(
    kindr::minimal::QuatTransformationTemplate<float>* T_object_detection_init) {
  ros::WallTime time_start = ros::WallTime::now();

  if (detection_pointcloud_.size() < 3) {
    LOG(WARNING) << "Detection PCA not possible! Too few points: "
                 << detection_pointcloud_.size();
    return false;
  }
  if (object_pointcloud_.size() < 3) {
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

bool DetectionMatcher::performICP(
    const kindr::minimal::QuatTransformationTemplate<float>& T_object_detection_init,
    kindr::minimal::QuatTransformationTemplate<float>* T_object_detection) {
  CHECK(T_object_detection);
  ros::WallTime time_start = ros::WallTime::now();

  // setup data points
  PointMatcher<float>::DataPoints points_object =
      PointMatcher_ros::rosMsgToPointMatcherCloud<float>(object_pointcloud_msg_);
  PointMatcher<float>::DataPoints points_detection =
      PointMatcher_ros::rosMsgToPointMatcherCloud<float>(detection_pointcloud_msg_);

  // setup icp
  PointMatcher<float>::ICP icp;
  icp.setDefault();

  std::string name;
  PointMatcherSupport::Parametrizable::Parameters params;

  // Prepare reading filters
  name = "MaxPointCountDataPointsFilter";
  params["maxCount"] = std::to_string(num_points_icp_);
  std::shared_ptr<PointMatcher<float>::DataPointsFilter> maxCount_read =
      PointMatcher<float>::get().DataPointsFilterRegistrar.create(name, params);
  params.clear();

  // Prepare reference filters
  name = "MaxPointCountDataPointsFilter";
  params["maxCount"] = std::to_string(num_points_icp_);
  std::shared_ptr<PointMatcher<float>::DataPointsFilter> maxCount_ref =
      PointMatcher<float>::get().DataPointsFilterRegistrar.create(name, params);
  params.clear();

  // Build ICP solution
  icp.readingDataPointsFilters.push_back(maxCount_read);
  icp.referenceDataPointsFilters.push_back(maxCount_ref);

  // icp: reference - object mesh, data - detection cloud
  PointMatcher<float>::TransformationParameters T_object_detection_icp;
  try {
    T_object_detection_icp =
        icp(points_detection, points_object, T_object_detection_init.getTransformationMatrix());
  } catch (PointMatcher<float>::ConvergenceError& error_msg) {
    LOG(WARNING) << "[DetectionMatcher] ICP was not successful!\n"
                    "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx";
    return false;
  }

  kindr::minimal::QuatTransformationTemplate<float>::TransformationMatrix Tmatrix(
      T_object_detection_icp);
  *T_object_detection =
      kindr::minimal::QuatTransformationTemplate<float>(Tmatrix);

  LOG(INFO) << "Time ICP: " << (ros::WallTime::now() - time_start).toSec();
  LOG(INFO) << "[DetectionMatcher] ICP on detection pointcloud "
               "and object mesh vertices successful!";
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