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

ObjectDetector3D::ObjectDetector3D(const ros::NodeHandle& nh,
                                   const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      pointcloud_topic_("/camera/depth/color/points"),
      object_frame_id_("object_detection_mesh"),
      num_points_icp_(500) {
  getParamsFromRos();
  subscribeToTopics();
  advertiseTopics();

  std::string off_file =
      nh_private.param<std::string>("off_model", "fail");
  if (!cgal::MeshModel::create(off_file, &mesh_model_)) {
    LOG(ERROR) << "Could not get mesh model from off file at "
               << off_file << "!";
  }
  LOG(INFO) << "Object mesh with "
            << mesh_model_->getMesh().size_of_facets()
            << " facets and " << mesh_model_->getMesh().size_of_vertices()
            << " vertices";

  processObject();
}

void ObjectDetector3D::getParamsFromRos() {
  nh_private_.param("pointcloud_topic", pointcloud_topic_, pointcloud_topic_);
  nh_private_.param("object_frame_id",
                    object_frame_id_, object_frame_id_);
  nh_private_.param("num_points_icp", num_points_icp_, num_points_icp_);
}

void ObjectDetector3D::subscribeToTopics() {
  int queue_size = 1;
  nh_private_.param("queue_size", queue_size, queue_size);
  detection_pointcloud_sub_ =
      nh_.subscribe(pointcloud_topic_, queue_size,
                    &ObjectDetector3D::objectDetectionCallback, this);
  LOG(INFO) << "Subscribed to pointcloud topic ["
            << detection_pointcloud_sub_.getTopic() << "]";
}

void ObjectDetector3D::advertiseTopics() {
  object_pointcloud_pub_ =
      nh_private_.advertise<sensor_msgs::PointCloud2>("object_pcl", 1, true);
  LOG(INFO) << "Publishing object poincloud to topic ["
            << object_pointcloud_pub_.getTopic() << "]";
  object_mesh_pub_ =
      nh_private_.advertise<cgal_msgs::TriangleMeshStamped>("object_mesh", 1,
                                                            true);
  LOG(INFO) << "Publishing object mesh to topic ["
            << object_mesh_pub_.getTopic() << "]";
  object_mesh_init_pub_ =
      nh_private_.advertise<cgal_msgs::TriangleMeshStamped>(
          "object_mesh_init", 1, true);
  LOG(INFO) << "Publishing init object mesh to topic ["
            << object_mesh_init_pub_.getTopic() << "]";
}

void ObjectDetector3D::processObject() {
  // TODO(gasserl): find appropriate number of points to sample
  int num_points_object_pointcloud = 1e3;
  nh_private_.param("num_points_object_pointcloud",
                    num_points_object_pointcloud,
                    num_points_object_pointcloud);
  cpt_utils::sample_pc_from_mesh(mesh_model_->getMesh(),
                                 num_points_object_pointcloud, 0.0,
                                 &object_pointcloud_);
  LOG(INFO) << "Converted object mesh with "
            << mesh_model_->getMesh().size_of_facets() << " facets and "
            << mesh_model_->getMesh().size_of_vertices()
            << " vertices to a pointcloud with "
            << object_pointcloud_.size() << " points";

  // Serialize to a ROS message
  pcl::toROSMsg(object_pointcloud_, object_pointcloud_msg_);

  // Visualize object
  bool visualize_object_on_startup = false;
  nh_private_.param("visualize_object_on_startup",
                    visualize_object_on_startup,
                    visualize_object_on_startup);
  if (visualize_object_on_startup) {
    visualizeObjectPointcloud(ros::Time::now(), detection_frame_id_);
    visualizeObjectMesh(detection_frame_id_, object_mesh_init_pub_);
    LOG(INFO) << "Visualizing object";
  }
}

void ObjectDetector3D::objectDetectionCallback(
    const sensor_msgs::PointCloud2 &cloud_msg_in) {
  detection_frame_id_ = cloud_msg_in.header.frame_id;
  detection_pointcloud_msg_ = cloud_msg_in;
  pcl::fromROSMsg(detection_pointcloud_msg_, detection_pointcloud_);

  processPointcloudUsingPcaAndIcp();
}

void ObjectDetector3D::processPointcloudUsingPcaAndIcp() {
  ros::WallTime time_start = ros::WallTime::now();

  // get initial guess
  Transformation T_object_detection_init;
  if(!findInitialGuessUsingPca(&T_object_detection_init)) {
    LOG(WARNING) << "Initialization of ICP from detection pointcloud to "
                    "object pointcloud failed!";
    T_object_detection_init.setIdentity();
  }
  publishTransformation(T_object_detection_init.inverse(),
                        detection_pointcloud_msg_.header.stamp,
                        detection_frame_id_, object_frame_id_ + "_init");
  visualizeObjectMesh(object_frame_id_ + "_init", object_mesh_init_pub_);

  // ICP with initial guess
  Transformation T_object_detection;
  if(!performICP(T_object_detection_init, &T_object_detection)) {
    LOG(WARNING) << "ICP from detection pointcloud to "
                    "object pointcloud failed!";
    T_object_detection.setIdentity();
  }

  // Publish results
  publishTransformation(T_object_detection.inverse(),
                        detection_pointcloud_msg_.header.stamp,
                        detection_frame_id_, object_frame_id_);
  visualizeObjectPointcloud(detection_pointcloud_msg_.header.stamp,
                            detection_frame_id_);
  visualizeObjectMesh(object_frame_id_, object_mesh_pub_);
  LOG(INFO) << "Total matching time: "
            << (ros::WallTime::now() - time_start).toSec();
}

bool ObjectDetector3D::findInitialGuessUsingPca(
    Transformation* T_object_detection_init) {
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
  // Get rotation between detection and object pointcloud
  Eigen::Matrix3f rotation_matrix =
      detection_vectors * object_vectors.transpose();

  Quaternion rotation;
  rotation.setIdentity();
  if (Quaternion::isValidRotationMatrix(rotation_matrix)) {
    rotation = Quaternion(rotation_matrix);
  } else {
    LOG(WARNING) << "Rotation matrix is not valid!";
    LOG(INFO) << "determinant: " << rotation_matrix.determinant();
    LOG(INFO) << "R*R^T:\n" << rotation_matrix * rotation_matrix.transpose();
    return false;
  }

  *T_object_detection_init = Transformation(rotation, translation).inverse();
  LOG(INFO) << "Time initial guess: "
            << (ros::WallTime::now() - time_start).toSec();
  return true;
}

bool ObjectDetector3D::performICP(const Transformation& T_object_detection_init,
                                  Transformation* T_object_detection) {
  CHECK(T_object_detection);
  ros::WallTime time_start = ros::WallTime::now();

  // setup data points
  PM::DataPoints points_object =
      PointMatcher_ros::rosMsgToPointMatcherCloud<float>(
          object_pointcloud_msg_);
  PM::DataPoints points_detection =
      PointMatcher_ros::rosMsgToPointMatcherCloud<float>(
          detection_pointcloud_msg_);

  // setup icp
  PM::ICP icp;
  icp.setDefault();

  // Prepare filters
  std::string name;
  PointMatcherSupport::Parametrizable::Parameters params;
  // Reading filters
  name = "MaxPointCountDataPointsFilter";
  params["maxCount"] = std::to_string(num_points_icp_);
  std::shared_ptr<PM::DataPointsFilter> maxCount_read =
      PM::get().DataPointsFilterRegistrar.create(name, params);
  params.clear();
  // Reference filters
  name = "MaxPointCountDataPointsFilter";
  params["maxCount"] = std::to_string(num_points_icp_);
  std::shared_ptr<PM::DataPointsFilter> maxCount_ref =
      PM::get().DataPointsFilterRegistrar.create(name, params);
  params.clear();

  // Build ICP solution
  icp.readingDataPointsFilters.push_back(maxCount_read);
  icp.referenceDataPointsFilters.push_back(maxCount_ref);

  // icp: reference - object mesh, data - detection cloud
  PM::TransformationParameters T_object_detection_icp;
  try {
    T_object_detection_icp =
        icp(points_detection, points_object,
            T_object_detection_init.getTransformationMatrix());
  } catch (PM::ConvergenceError& error_msg) {
    LOG(WARNING) << "ICP was not successful!\n"
                    "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx";
    return false;
  }

  Transformation::TransformationMatrix Tmatrix(
      T_object_detection_icp);
  *T_object_detection =
      Transformation(Tmatrix);

  LOG(INFO) << "Time ICP: " << (ros::WallTime::now() - time_start).toSec();
  LOG(INFO) << "ICP on detection pointcloud "
               "and object mesh vertices successful!";
  return true;
}

void ObjectDetector3D::publishTransformation(const Transformation& transform,
                                             const ros::Time& stamp,
                                             const std::string& parent_frame_id,
                                             const std::string& child_frame_id) {
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

void ObjectDetector3D::visualizeObjectMesh(
    const std::string& frame_id, const ros::Publisher& publisher) const {
  cgal_msgs::TriangleMeshStamped p_msg;

  // triangle mesh to prob. msg
  cgal_msgs::TriangleMesh t_msg;
  cgal::Polyhedron mesh = mesh_model_->getMesh();
  cgal::triangleMeshToMsg(mesh, &t_msg);
  p_msg.mesh = t_msg;

  p_msg.header.frame_id = frame_id;
  p_msg.header.stamp = detection_pointcloud_msg_.header.stamp;
  p_msg.header.seq = 0;
  publisher.publish(p_msg);
}

void ObjectDetector3D::visualizeObjectPointcloud(const ros::Time& timestamp,
                                                 const std::string& frame_id) {
  detection_pointcloud_msg_.header.stamp = timestamp;
  object_pointcloud_msg_.header.frame_id = frame_id;
  object_pointcloud_pub_.publish(object_pointcloud_msg_);
}

}
}