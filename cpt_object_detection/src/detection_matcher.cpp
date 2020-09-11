#include "cpt_object_detection/detection_matcher.h"

#include <cgal_conversions/mesh_conversions.h>
#include <cgal_msgs/TriangleMeshStamped.h>
#include <cpt_utils/pc_processing.h>
#include <geometry_msgs/TransformStamped.h>
#include <minkindr_conversions/kindr_msg.h>
#include <pcl/common/pca.h>
#include <pointmatcher_ros/point_cloud.h>
#include <tf/transform_broadcaster.h>

namespace cad_percept {
namespace object_detection {

ObjectDetector3D::ObjectDetector3D(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      object_frame_id_("object_detection_mesh"),
      pointcloud_topic_("/camera/depth/color/points") {
  getParamsFromRos();
  subscribeToTopics();
  advertiseTopics();

  const std::string& off_file = nh_private.param<std::string>("off_model", "fail");
  if (!cgal::MeshModel::create(off_file, &mesh_model_)) {
    LOG(ERROR) << "Could not get mesh model from off file at " << off_file << "!";
  }
  LOG(INFO) << "Object mesh with " << mesh_model_->getMesh().size_of_facets() << " facets and "
            << mesh_model_->getMesh().size_of_vertices() << " vertices";

  // Visualize object
  bool visualize_object_on_startup = false;
  nh_private_.param("visualize_object_on_startup", visualize_object_on_startup,
                    visualize_object_on_startup);
  if (visualize_object_on_startup) {
    visualizeMesh(mesh_model_, ros::Time::now(), detection_frame_id_, object_mesh_init_pub_);
    LOG(INFO) << "Visualizing object";
  }
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
  object_mesh_pub_ = nh_private_.advertise<cgal_msgs::TriangleMeshStamped>("object_mesh", 1, true);
  LOG(INFO) << "Publishing object mesh to topic [" << object_mesh_pub_.getTopic() << "]";
  object_mesh_init_pub_ =
      nh_private_.advertise<cgal_msgs::TriangleMeshStamped>("object_mesh_init", 1, true);
  LOG(INFO) << "Publishing init object mesh to topic [" << object_mesh_init_pub_.getTopic() << "]";
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
      mesh_model_, detection_pointcloud_, icp_config_file_, &T_object_detection_init);

  // Publish transformations to TF
  publishTransformation(T_object_detection_init.inverse(), detection_stamp_, detection_frame_id_,
                        object_frame_id_ + "_init");
  publishTransformation(T_object_detection.inverse(), detection_stamp_, detection_frame_id_,
                        object_frame_id_);

  // Visualize object
  visualizeMesh(mesh_model_, detection_stamp_, object_frame_id_ + "_init", object_mesh_init_pub_);
  visualizeMesh(mesh_model_, detection_stamp_, object_frame_id_, object_mesh_pub_);
}

ObjectDetector3D::Transformation ObjectDetector3D::alignDetectionUsingPcaAndIcp(
    const cgal::MeshModel::Ptr& mesh_model,
    const pcl::PointCloud<pcl::PointXYZ>& detection_pointcloud, const std::string& config_file,
    Transformation* T_object_detection_init) {
  CHECK(T_object_detection_init);
  ros::WallTime time_start = ros::WallTime::now();

  // Get initial guess with PCA
  Transformation T_detection_object_pca = pca(mesh_model, detection_pointcloud);
  *T_object_detection_init = T_detection_object_pca.inverse();

  // Get final alignment with ICP
  Transformation T_object_detection =
      icp(mesh_model, detection_pointcloud, *T_object_detection_init, config_file);

  LOG(INFO) << "Time matching total: " << (ros::WallTime::now() - time_start).toSec() << " s";
  return T_object_detection;
}

ObjectDetector3D::Transformation ObjectDetector3D::alignDetectionUsingPcaAndIcp(
    const cgal::MeshModel::Ptr& mesh_model,
    const pcl::PointCloud<pcl::PointXYZ>& detection_pointcloud) {
  Transformation T;
  std::string config_file;
  return alignDetectionUsingPcaAndIcp(mesh_model, detection_pointcloud, config_file, &T);
}

ObjectDetector3D::Transformation ObjectDetector3D::alignDetectionUsingPcaAndIcp(
    const cgal::MeshModel::Ptr& mesh_model,
    const pcl::PointCloud<pcl::PointXYZ>& detection_pointcloud, const std::string& config_file) {
  Transformation T;
  return alignDetectionUsingPcaAndIcp(mesh_model, detection_pointcloud, config_file, &T);
}

ObjectDetector3D::Transformation ObjectDetector3D::pca(
    const cgal::MeshModel::Ptr& mesh_model,
    const pcl::PointCloud<pcl::PointXYZ>& detection_pointcloud) {
  CHECK_NOTNULL(mesh_model);
  ros::WallTime time_start = ros::WallTime::now();

  if (detection_pointcloud.size() < 3) {
    LOG(WARNING) << "Detection PCA not possible! Too few points: " << detection_pointcloud.size();
    return Transformation();
  }
  if (mesh_model->getMesh().empty()) {
    LOG(WARNING) << "Object PCA not possible! Mesh is empty.";
    return Transformation();
  }

  // Get data from detection pointcloud
  pcl::PCA<pcl::PointXYZ> pca_detection;
  pcl::PointCloud<pcl::PointXYZ>::Ptr detection_pointcloud_ptr = detection_pointcloud.makeShared();
  pca_detection.setInputCloud(detection_pointcloud_ptr);
  Eigen::Vector4f detection_centroid = pca_detection.getMean();
  Eigen::Matrix3f detection_vectors = pca_detection.getEigenVectors();

  // Get coordinate system to be right
  if (detection_vectors.determinant() < 0) {
    detection_vectors.col(2) = -detection_vectors.col(2);
  }

  // Get triangle vector from mesh
  std::vector<CGAL::Simple_cartesian<double>::Triangle_3> triangles;
  triangles.reserve(mesh_model->size());
  for (const auto& id : mesh_model->getFacetIds()) {
    triangles.push_back(mesh_model->getTriangle(id));
  }

  // Compute PCA for object triangles
  CGAL::Simple_cartesian<double>::Plane_3 plane;
  CGAL::Simple_cartesian<double>::Point_3 object_centroid;
  CGAL::linear_least_squares_fitting_3(triangles.begin(), triangles.end(), plane, object_centroid,
                                       CGAL::Dimension_tag<2>());

  // Get coordinate system from PCA
  Eigen::Matrix3f object_vectors;
  object_vectors.col(0) =
      Eigen::Vector3f(plane.base1().x(), plane.base1().y(), plane.base1().z()).normalized();
  object_vectors.col(1) =
      Eigen::Vector3f(plane.base2().x(), plane.base2().y(), plane.base2().z()).normalized();
  object_vectors.col(2) =
      Eigen::Vector3f(plane.orthogonal_vector().x(), plane.orthogonal_vector().y(),
                      plane.orthogonal_vector().z())
          .normalized();

  // Translation from mean of pointclouds det_r_obj_det
  kindr::minimal::PositionTemplate<float> translation(
      detection_centroid.head(3) -
      Eigen::Vector3f(object_centroid.x(), object_centroid.y(), object_centroid.z()));

  // Get rotation between detection and object pointcloud
  Eigen::Matrix3f rotation_matrix = detection_vectors * object_vectors.transpose();

  Quaternion rotation;
  rotation.setIdentity();
  if (Quaternion::isValidRotationMatrix(rotation_matrix)) {
    rotation = Quaternion(rotation_matrix);
  } else {
    LOG(WARNING) << "Rotation matrix is not valid!";
    LOG(INFO) << "determinant: " << rotation_matrix.determinant();
    LOG(INFO) << "R*R^T - I:\n"
              << rotation_matrix * rotation_matrix.transpose() - Eigen::Matrix3f::Identity();
    return Transformation();
  }

  LOG(INFO) << "Time PCA: " << (ros::WallTime::now() - time_start).toSec() << " s";
  return Transformation(rotation, translation);
}

ObjectDetector3D::PM::DataPoints ObjectDetector3D::convertMeshToDataPoints(
    const cgal::MeshModel::Ptr& mesh_model) {
  CHECK(mesh_model);

  // Define feature and descriptor labels
  PM::DataPoints::Labels feature_labels;
  feature_labels.push_back(PM::DataPoints::Label("x", 1));
  feature_labels.push_back(PM::DataPoints::Label("y", 1));
  feature_labels.push_back(PM::DataPoints::Label("z", 1));
  feature_labels.push_back(PM::DataPoints::Label("pad", 1));

  PM::DataPoints::Labels descriptor_labels;
  descriptor_labels.push_back(PM::DataPoints::Label("normals", 3));

  // Get data from mesh
  PM::Matrix features(feature_labels.totalDim(), mesh_model->size());
  PM::Matrix descriptors(descriptor_labels.totalDim(), mesh_model->size());
  size_t i = 0;
  ros::WallTime conversion_start = ros::WallTime::now();
  for (const auto& id : mesh_model->getFacetIds()) {
    CGAL::Simple_cartesian<double>::Triangle_3 triangle = mesh_model->getTriangle(id);
    CGAL::Simple_cartesian<double>::Point_3 centroid =
        CGAL::centroid(triangle);  // TODO(gasserl): replace with just a vertex?
    CGAL::Simple_cartesian<double>::Vector_3 normal =
        triangle.supporting_plane().orthogonal_vector();

    features.col(i) = Eigen::Vector4f(centroid.x(), centroid.y(), centroid.z(), 1);
    descriptors.col(i) = Eigen::Vector3f(normal.x(), normal.y(), normal.z());
    ++i;
  }
  LOG(INFO) << "Time conversion mesh to pointmatcher: "
            << (ros::WallTime::now() - conversion_start).toSec() << " s";

  return PM::DataPoints(features, feature_labels, descriptors, descriptor_labels);
}

ObjectDetector3D::Transformation ObjectDetector3D::icp(
    const cgal::MeshModel::Ptr& mesh_model,
    const pcl::PointCloud<pcl::PointXYZ>& detection_pointcloud,
    const Transformation& T_object_detection_init, const std::string& config_file) {
  CHECK(mesh_model);
  ros::WallTime time_start = ros::WallTime::now();

  // setup data points
  PM::DataPoints points_object = convertMeshToDataPoints(mesh_model);

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(detection_pointcloud, msg);
  PM::DataPoints points_detection = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(msg);

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
    LOG(INFO) << "No ICP config file given, using default ICP settings";
    icp.setDefault();
  }

  // icp: reference - object mesh, data - detection cloud
  PM::TransformationParameters T_object_detection_icp;
  try {
    T_object_detection_icp = icp.compute(points_detection, points_object,
                                         T_object_detection_init.getTransformationMatrix());
  } catch (PM::ConvergenceError& error_msg) {
    LOG(WARNING) << "ICP was not successful!";
    return T_object_detection_init;
  }

  if (icp.getMaxNumIterationsReached()) {
    LOG(ERROR) << "ICP reached maximum number of iterations!";
  }

  if (!Quaternion::isValidRotationMatrix(T_object_detection_icp.block<3, 3>(0, 0))) {
    LOG(ERROR) << "Invalid rotation matrix!";
    return T_object_detection_init;
  }
  Transformation::TransformationMatrix Tmatrix(T_object_detection_icp);

  LOG(INFO) << "ICP on detection pointcloud and object mesh vertices successful!";
  LOG(INFO) << "Time ICP: " << (ros::WallTime::now() - time_start).toSec() << " s";
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