#include "cpt_object_detection/detection_matcher.h"

#include <geometry_msgs/TransformStamped.h>
#include <cgal_msgs/TriangleMeshStamped.h>
#include <cgal_conversions/mesh_conversions.h>
#include <tf/transform_broadcaster.h>
#include <pointmatcher_ros/point_cloud.h>
#include <pcl/common/pca.h>
#include <cpt_utils/pc_processing.h>
#include <minkindr_conversions/kindr_msg.h>

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
  nh_private_.param("icp_config_file", icp_config_file_,
                    icp_config_file_);
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
  Transformation T_detection_object_pca = pca(object_pointcloud_,
                                                     detection_pointcloud_);
  // ICP with initial guess
  Transformation T_object_detection =
      icp(object_pointcloud_, detection_pointcloud_,
          T_detection_object_pca.inverse(), icp_config_file_);

  publishTransformation(T_detection_object_pca,
                        detection_pointcloud_msg_.header.stamp,
                        detection_frame_id_, object_frame_id_ + "_init");
  visualizeObjectMesh(object_frame_id_ + "_init", object_mesh_init_pub_);

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

ObjectDetector3D::Transformation ObjectDetector3D::pca(
    const pcl::PointCloud<pcl::PointXYZ>& object_pointcloud,
    const pcl::PointCloud<pcl::PointXYZ>& detection_pointcloud) {
  ros::WallTime time_start = ros::WallTime::now();

  if (detection_pointcloud.size() < 3) {
    LOG(WARNING) << "Detection PCA not possible! Too few points: "
                 << detection_pointcloud.size();
    return Transformation();
  }
  if (object_pointcloud.size() < 3) {
    LOG(WARNING) << "Object PCA not possible! Too few points: "
                 << object_pointcloud.size();
    return Transformation();
  }

  // Get data from detection pointcloud
  pcl::PCA<pcl::PointXYZ> pca_detection;
  pcl::PointCloud<pcl::PointXYZ>::Ptr detection_pointcloud_ptr =
      detection_pointcloud.makeShared();
  pca_detection.setInputCloud(detection_pointcloud_ptr);
  Eigen::Vector4f detection_centroid = pca_detection.getMean();
  Eigen::Matrix3f detection_vectors = pca_detection.getEigenVectors();

  // Get data from object pointcloud
  pcl::PCA<pcl::PointXYZ> pca_object;
  pcl::PointCloud<pcl::PointXYZ>::Ptr object_pointcloud_ptr =
      object_pointcloud.makeShared();
  pca_object.setInputCloud(object_pointcloud_ptr);
  Eigen::Vector4f object_centroid = pca_object.getMean();
  Eigen::Matrix3f object_vectors = pca_object.getEigenVectors();

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
    return Transformation();
  }

  LOG(INFO) << "Time PCA: " << (ros::WallTime::now() - time_start).toSec();
  return Transformation(rotation, translation);
}

ObjectDetector3D::Transformation ObjectDetector3D::icp(
    const pcl::PointCloud<pcl::PointXYZ>& object_pointcloud,
    const pcl::PointCloud<pcl::PointXYZ>& detection_pointcloud,
    const Transformation& T_object_detection_init,
    const std::string& config_file) {
  ros::WallTime time_start = ros::WallTime::now();

  // setup data points
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(object_pointcloud, msg);
  PM::DataPoints points_object =
      PointMatcher_ros::rosMsgToPointMatcherCloud<float>(msg);
  pcl::toROSMsg(detection_pointcloud, msg);
  PM::DataPoints points_detection =
      PointMatcher_ros::rosMsgToPointMatcherCloud<float>(msg);

  // setup icp
  PM::ICP icp;
  if (!config_file.empty()) {
    std::ifstream ifs(config_file.c_str());
    if (ifs.good()) {
      icp.loadFromYaml(ifs);
    } else {
      LOG(ERROR) << "Cannot load ICP config from YAML file " << config_file;
      icp.setDefault();
    }
  } else {
    LOG(INFO) << "No ICP config file given, using default";
    icp.setDefault();
  }

  // icp: reference - object mesh, data - detection cloud
  PM::TransformationParameters T_object_detection_icp;
  try {
    T_object_detection_icp =
        icp(points_detection, points_object,
            T_object_detection_init.getTransformationMatrix());
  } catch (PM::ConvergenceError& error_msg) {
    LOG(WARNING) << "ICP was not successful!";
    return T_object_detection_init;
  }

  if (!Quaternion::isValidRotationMatrix(
          T_object_detection_icp.block<3,3>(0,0))) {
    LOG(ERROR) << "Invalid rotation matrix!";
    return T_object_detection_init;
  }
  Transformation::TransformationMatrix Tmatrix(T_object_detection_icp);

  LOG(INFO) << "Time ICP: " << (ros::WallTime::now() - time_start).toSec();
  return Transformation(Tmatrix);
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