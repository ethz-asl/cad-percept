#include "cpt_object_detection/detection_matcher.h"

#include <CGAL/linear_least_squares_fitting_3.h>
#include <cgal_conversions/mesh_conversions.h>
#include <cgal_msgs/TriangleMeshStamped.h>
#include <cpt_utils/pc_processing.h>
#include <geometry_msgs/TransformStamped.h>
#include <minkindr_conversions/kindr_msg.h>
#include <modelify/feature_toolbox/descriptor_toolbox_3d.h>
#include <modelify/feature_toolbox/keypoint_toolbox_3d.h>
#include <modelify/pcl_common.h>
#include <modelify/registration_toolbox/fast_global_registration.h>
#include <modelify/registration_toolbox/registration_toolbox.h>
#include <pcl/common/pca.h>
#include <pointmatcher_ros/point_cloud.h>
#include <teaser/registration.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace cad_percept {
namespace object_detection {

ObjectDetector3D::ObjectDetector3D(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      object_frame_id_("object_detection_mesh"),
      pointcloud_topic_("/camera/depth/color/points"),
      detection_frame_id_("camera_depth_optical_frame"),
      keypoint_type_(kHarris),
      descriptor_type_(kShot),
      matching_method_(kConventional),
      use_3d_features_(true),
      refine_(true),
      correspondence_threshold_(0.1),
      downsampling_resolution_(0.001) {
  getParamsFromRos();
  subscribeToTopics();
  advertiseTopics();

  const std::string& off_file = nh_private.param<std::string>("off_model", "fail");
  if (!cgal::MeshModel::create(off_file, &mesh_model_)) {
    LOG(ERROR) << "Could not get mesh model from off file at " << off_file << "!";
  }
  LOG(INFO) << "Object mesh with " << mesh_model_->getMesh().size_of_facets() << " facets and "
            << mesh_model_->getMesh().size_of_vertices() << " vertices";

  processMesh();
}

void ObjectDetector3D::getParamsFromRos() {
  nh_private_.param("pointcloud_topic", pointcloud_topic_, pointcloud_topic_);
  nh_private_.param("object_frame_id", object_frame_id_, object_frame_id_);
  nh_private_.param("use_3d_features", use_3d_features_, use_3d_features_);
  nh_private_.param("refine", refine_, refine_);
  nh_private_.param("icp_config_file", icp_config_file_, icp_config_file_);
  nh_private_.param("correspondence_threshold", correspondence_threshold_,
                    correspondence_threshold_);
  nh_private_.param("downsampling", downsampling_resolution_, downsampling_resolution_);

  std::string keypoint_type;
  nh_private_.param("keypoint_type", keypoint_type, keypoint_type);
  bool valid_keypoint_type = false;
  for (int i = 0; i < kNumKeypointTypes; ++i) {
    if (keypoint_type == KeypointNames[i]) {
      keypoint_type_ = static_cast<KeypointType>(i);
      valid_keypoint_type = true;
      break;
    }
  }
  if (!valid_keypoint_type) {
    LOG(ERROR) << "Unknown keypoint type! " << keypoint_type;
    LOG(INFO) << "Keypoint types:";
    for (int i = 0; i < KeypointType::kNumKeypointTypes; ++i) {
      LOG(INFO) << KeypointNames[i];
    }
  }
  LOG(INFO) << "Using keypoint type " << KeypointNames[keypoint_type_];

  std::string descriptor_type;
  nh_private_.param("descriptor_type", descriptor_type, descriptor_type);
  bool valid_descriptor_type = false;
  for (int i = 0; i < kNumDescriptorTypes; ++i) {
    if (descriptor_type == DescriptorNames[i]) {
      descriptor_type_ = static_cast<DescriptorType>(i);
      valid_descriptor_type = true;
      break;
    }
  }
  if (!valid_descriptor_type && !descriptor_type.empty()) {
    LOG(ERROR) << "Unknown descriptor type! " << descriptor_type;
    LOG(INFO) << "Descriptor types:";
    for (int i = 0; i < DescriptorType::kNumDescriptorTypes; ++i) {
      LOG(INFO) << DescriptorNames[i];
    }
  }
  LOG(INFO) << "Using descriptor type " << DescriptorNames[descriptor_type_];

  std::string matching_method;
  nh_private_.param("matching_method", matching_method, matching_method);
  bool valid_matching_method = false;
  for (int i = 0; i < kNumMatchingMethods; ++i) {
    if (matching_method == MatchingMethodNames[i]) {
      matching_method_ = static_cast<MatchingMethod>(i);
      valid_matching_method = true;
      break;
    }
  }
  if (!valid_matching_method && !matching_method.empty()) {
    LOG(ERROR) << "Unknown matching method! " << matching_method;
    LOG(INFO) << "Matching methods:";
    for (int i = 0; i < MatchingMethod::kNumMatchingMethods; ++i) {
      LOG(INFO) << MatchingMethodNames[i];
    }
  }
  LOG(INFO) << "Using matching method " << MatchingMethodNames[matching_method_];
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

  object_keypoint_pub_ =
      nh_private_.advertise<sensor_msgs::PointCloud2>("object_keypoints", 1, true);
  LOG(INFO) << "[ObjectDetector3D] Publishing object_keypoints to topic ["
            << object_keypoint_pub_.getTopic() << "]";
  detection_keypoint_pub_ =
      nh_private_.advertise<sensor_msgs::PointCloud2>("detection_keypoints", 1, true);
  LOG(INFO) << "[ObjectDetector3D] Publishing detection_keypoints to topic ["
            << detection_keypoint_pub_.getTopic() << "]";

  correspondences_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>("correspondences", 1, true);
  normals_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("surface_normals", 1, true);
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

  // Get 3D features of object pointcloud
  object_surfels_.reset(new modelify::PointSurfelCloudType());
  object_keypoints_.reset(new modelify::PointSurfelCloudType());
  switch (descriptor_type_) {
    case kFpfh:
      object_descriptors_fpfh_.reset(new modelify::DescriptorFPFHCloudType());
      get3dFeatures<modelify::DescriptorFPFH>(object_pointcloud_, object_surfels_,
                                              object_keypoints_, object_descriptors_fpfh_);
      break;
    case kShot:
      object_descriptors_shot_.reset(new modelify::DescriptorSHOTCloudType());
      get3dFeatures<modelify::DescriptorSHOT>(object_pointcloud_, object_surfels_,
                                              object_keypoints_, object_descriptors_shot_);
      break;
    default:
      LOG(ERROR) << "Unknown descriptor type! " << descriptor_type_;
      LOG(INFO) << "Descriptor types:";
      for (int i = 0; i < DescriptorType::kNumDescriptorTypes; ++i) {
        LOG(INFO) << i << " (" << DescriptorNames[i] << ")";
      }
      break;
  }
  LOG(INFO) << "Obtained 3D features of object pointcloud";

  // Visualize object
  bool visualize_object_on_startup = false;
  nh_private_.param("visualize_object_on_startup", visualize_object_on_startup,
                    visualize_object_on_startup);
  if (visualize_object_on_startup) {
    visualizePointcloud(object_pointcloud_, ros::Time::now(), detection_frame_id_,
                        object_pointcloud_pub_);
    visualizeMesh(mesh_model_, ros::Time::now(), detection_frame_id_, object_mesh_init_pub_);
    visualizeKeypoints(object_keypoints_, ros::Time::now(), detection_frame_id_,
                       object_keypoint_pub_);
    LOG(INFO) << "Visualizing object";
  }
}

void ObjectDetector3D::objectDetectionCallback(const sensor_msgs::PointCloud2& cloud_msg_in) {
  detection_frame_id_ = cloud_msg_in.header.frame_id;
  detection_stamp_ = cloud_msg_in.header.stamp;
  pcl::fromROSMsg(cloud_msg_in, detection_pointcloud_);

  if (!use_3d_features_) {
    processDetectionUsingPcaAndIcp();
  } else {
    processDetectionUsing3dFeatures();
  }
}

// TODO(gasserl): make child classes for each thing?
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
  visualizePointcloud(object_pointcloud_, detection_stamp_, detection_frame_id_,
                      object_pointcloud_pub_);
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

// TODO(gasserl): another child class?
void ObjectDetector3D::processDetectionUsing3dFeatures() {
  // Downsampling detection pointcloud
  if (downsampling_resolution_ > 0) {
    pcl::PointCloud<pcl::PointXYZ> temp(detection_pointcloud_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr detection_pointcloud_ptr =
        boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(temp);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
    voxel_grid_filter.setInputCloud(detection_pointcloud_ptr);
    voxel_grid_filter.setLeafSize(downsampling_resolution_, downsampling_resolution_,
                                  downsampling_resolution_);
    voxel_grid_filter.filter(detection_pointcloud_);
    LOG(INFO) << "Detection pointcloud downsampled to resolution of " << downsampling_resolution_
              << " m, resulting in " << detection_pointcloud_.size() << " points";
  }

  // Compute transform between detection and object
  Transformation T_features;
  modelify::PointSurfelCloudType::Ptr detection_surfels(new modelify::PointSurfelCloudType());
  modelify::PointSurfelCloudType::Ptr detection_keypoints(new modelify::PointSurfelCloudType());
  typename pcl::PointCloud<modelify::DescriptorFPFH>::Ptr detection_descriptors_fpfh(
      new pcl::PointCloud<modelify::DescriptorFPFH>());
  typename pcl::PointCloud<modelify::DescriptorSHOT>::Ptr detection_descriptors_shot(
      new pcl::PointCloud<modelify::DescriptorSHOT>());
  modelify::CorrespondencesTypePtr correspondences(new modelify::CorrespondencesType());
  switch (descriptor_type_) {
    case kFpfh:
      get3dFeatures<modelify::DescriptorFPFH>(detection_pointcloud_, detection_surfels,
                                              detection_keypoints, detection_descriptors_fpfh);
      T_features = computeTransformUsing3dFeatures<modelify::DescriptorFPFH>(
          matching_method_, detection_surfels, detection_keypoints, detection_descriptors_fpfh,
          object_surfels_, object_keypoints_, object_descriptors_fpfh_, correspondence_threshold_,
          correspondences);
      break;
    case kShot:
      get3dFeatures<modelify::DescriptorSHOT>(detection_pointcloud_, detection_surfels,
                                              detection_keypoints, detection_descriptors_shot);
      T_features = computeTransformUsing3dFeatures<modelify::DescriptorSHOT>(
          matching_method_, detection_surfels, detection_keypoints, detection_descriptors_shot,
          object_surfels_, object_keypoints_, object_descriptors_shot_, correspondence_threshold_,
          correspondences);
      break;
    default:
      LOG(ERROR) << "Unknown descriptor type! " << descriptor_type_;
      LOG(INFO) << "Descriptor types:";
      for (int i = 0; i < DescriptorType::kNumDescriptorTypes; ++i) {
        LOG(INFO) << i << " (" << DescriptorNames[i] << ")";
      }
      return;
  }

  if (!T_features.getTransformationMatrix().allFinite() ||
      !Quaternion::isValidRotationMatrix(T_features.getRotationMatrix())) {
    LOG(ERROR) << "Transformation from 3D features is not valid!\n" << T_features;
    return;
  }

  // Refine using ICP
  Transformation T_icp = T_features;
  if (refine_) {
    T_icp = icpUsingModelify(detection_surfels, object_surfels_, T_features);
  }

  if (!T_icp.getTransformationMatrix().allFinite() ||
      !Quaternion::isValidRotationMatrix(T_icp.getRotationMatrix())) {
    LOG(ERROR) << "Transformation from ICP is not valid!\n" << T_icp;
    return;
  }

  // Visualizations
  visualizeNormals(detection_surfels, "detection", detection_frame_id_, normals_pub_);
  visualizeNormals(object_surfels_, "object", detection_frame_id_, normals_pub_);
  visualizeKeypoints(detection_keypoints, detection_stamp_, detection_frame_id_,
                     detection_keypoint_pub_);
  visualizeKeypoints(object_keypoints_, detection_stamp_, detection_frame_id_,
                     object_keypoint_pub_);

  // Visualize correspondences
  visualizeCorrespondences(detection_keypoints, correspondences, detection_frame_id_,
                           correspondences_pub_);

  // Publish initial transform
  publishTransformation(T_features.inverse(), detection_stamp_, detection_frame_id_,
                        object_frame_id_ + "_init");
  visualizeMesh(mesh_model_, detection_stamp_, object_frame_id_ + "_init", object_mesh_init_pub_);

  // Publish results
  publishTransformation(T_icp.inverse(), detection_stamp_, detection_frame_id_, object_frame_id_);
  visualizePointcloud(object_pointcloud_, detection_stamp_, detection_frame_id_,
                      object_pointcloud_pub_);
  visualizeMesh(mesh_model_, detection_stamp_, object_frame_id_, object_mesh_pub_);
  LOG(INFO) << "Published results.";
}

template <typename descriptor_type>
ObjectDetector3D::Transformation ObjectDetector3D::computeTransformUsing3dFeatures(
    MatchingMethod matching_method, const modelify::PointSurfelCloudType::Ptr& detection_surfels,
    const modelify::PointSurfelCloudType::Ptr& detection_keypoints,
    const typename pcl::PointCloud<descriptor_type>::Ptr& detection_descriptors,
    const modelify::PointSurfelCloudType::Ptr& object_surfels,
    const modelify::PointSurfelCloudType::Ptr& object_keypoints,
    const typename pcl::PointCloud<descriptor_type>::Ptr& object_descriptors,
    double similarity_threshold, const modelify::CorrespondencesTypePtr& correspondences) {
  if (object_descriptors->empty()) {
    LOG(ERROR) << "Object pointcloud has no features!";
    return Transformation();
  }

  if (detection_descriptors->empty()) {
    LOG(ERROR) << "Detection pointcloud has no features!";
    return Transformation();
  }

  // Match features
  switch (matching_method) {
    case kConventional:
      return computeTransformUsingModelify<descriptor_type>(
          detection_keypoints, detection_descriptors, object_keypoints, object_descriptors,
          similarity_threshold, correspondences);
      break;
    case kFastGlobalRegistration:
      return computeTransformUsingFgr<descriptor_type>(
          detection_surfels, detection_keypoints, detection_descriptors, object_surfels,
          object_keypoints, object_descriptors, correspondences);
      break;
    case kTeaser:
      return computeTransformUsingTeaser<descriptor_type>(
          detection_keypoints, detection_descriptors, object_keypoints, object_descriptors,
          similarity_threshold, correspondences);
      break;
    default:
      LOG(ERROR) << "Unknown matching method! " << matching_method;
      return Transformation();
  }
}

template <typename descriptor_type>
ObjectDetector3D::Transformation ObjectDetector3D::computeTransformUsingFgr(
    const modelify::PointSurfelCloudType::Ptr& detection_surfels,
    const modelify::PointSurfelCloudType::Ptr& detection_keypoints,
    const typename pcl::PointCloud<descriptor_type>::Ptr& detection_descriptors,
    const modelify::PointSurfelCloudType::Ptr& object_surfels,
    const modelify::PointSurfelCloudType::Ptr& object_keypoints,
    const typename pcl::PointCloud<descriptor_type>::Ptr& object_descriptors,
    const modelify::CorrespondencesTypePtr& correspondences) {
  ros::WallTime start = ros::WallTime::now();
  modelify::registration_toolbox::FastGlobalRegistrationParams fgr_params;
  fgr_params.crosscheck_test = true;  // 300 -> 30
  fgr_params.tuple_test = false;      // 30 -> 0
  fgr_params.refine_using_icp = false;
  fgr_params.use_absolute_scale = true;
  modelify::Transformation transform;
  modelify::Correspondences corrs;
  if (!modelify::registration_toolbox::fast_global_registration::
          estimateTransformationFastGlobalRegistration<modelify::PointSurfelType, descriptor_type>(
              detection_surfels, object_surfels, detection_keypoints, object_keypoints,
              detection_descriptors, object_descriptors, fgr_params, &corrs, &transform)) {
    LOG(ERROR) << "Fast global registration was not successful!";
    return Transformation();
  }
  LOG(INFO) << "Time FGR: " << (ros::WallTime::now() - start).toSec();

  for (const modelify::CorrespondencePair& correspondence : corrs) {
    pcl::Correspondence corr;
    corr.index_match = correspondence.first;
    corr.index_query = correspondence.second;
    correspondences->push_back(corr);
  }

  return ObjectDetector3D::Transformation(transform);
}

template <typename descriptor_type>
ObjectDetector3D::Transformation ObjectDetector3D::computeTransformUsingModelify(
    const modelify::PointSurfelCloudType::Ptr& detection_keypoints,
    const typename pcl::PointCloud<descriptor_type>::Ptr& detection_descriptors,
    const modelify::PointSurfelCloudType::Ptr& object_keypoints,
    const typename pcl::PointCloud<descriptor_type>::Ptr& object_descriptors,
    double correspondence_threshold, const modelify::CorrespondencesTypePtr& correspondences) {
  // Find correspondences
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  modelify::registration_toolbox::FlannSearchMatchingParams flann_params;
  flann_params.num_of_correspondences = 3;
  if (object_descriptors->size() < flann_params.num_of_correspondences ||
      detection_descriptors->size() < flann_params.num_of_correspondences) {
    LOG(ERROR) << "Too few features found! Object: " << object_descriptors->size() << "/"
               << flann_params.num_of_correspondences
               << ", Detection: " << detection_descriptors->size() << "/"
               << flann_params.num_of_correspondences;
    return Transformation();
  }
  flann_params.similarity_threshold = correspondence_threshold;
  if (correspondence_threshold == 0) {
    flann_params.prefilter = false;
  }
  flann_params.keep_statistics = true;
  modelify::registration_toolbox::matchDescriptorsFlannSearch<descriptor_type>(
      detection_descriptors, object_descriptors, flann_params, correspondences);
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  LOG(INFO) << "Time correspondences: "
            << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() / 1e6;
  if (correspondences->empty()) {
    LOG(ERROR) << "No correspondences found!";
    return Transformation();
  }
  LOG(INFO) << "Found " << correspondences->size() << " correspondences.";

  // Get transformation between detection and object pointcloud
  return computeTransformFromCorrespondences(detection_keypoints, object_keypoints,
                                             correspondences);
}

template <typename descriptor_type>
ObjectDetector3D::Transformation ObjectDetector3D::computeTransformUsingTeaser(
    const modelify::PointSurfelCloudType::Ptr& detection_keypoints,
    const typename pcl::PointCloud<descriptor_type>::Ptr& detection_descriptors,
    const modelify::PointSurfelCloudType::Ptr& object_keypoints,
    const typename pcl::PointCloud<descriptor_type>::Ptr& object_descriptors,
    double correspondence_threshold, const modelify::CorrespondencesTypePtr& correspondences) {
  // Find correspondences
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  modelify::registration_toolbox::FlannSearchMatchingParams flann_params;
  flann_params.num_of_correspondences = 3;
  if (object_descriptors->size() < flann_params.num_of_correspondences ||
      detection_descriptors->size() < flann_params.num_of_correspondences) {
    LOG(ERROR) << "Too few features found! Object: " << object_descriptors->size() << "/"
               << flann_params.num_of_correspondences
               << ", Detection: " << detection_descriptors->size() << "/"
               << flann_params.num_of_correspondences;
    return Transformation();
  }
  flann_params.similarity_threshold = correspondence_threshold;
  if (correspondence_threshold == 0) {
    flann_params.prefilter = false;
  }
  flann_params.keep_statistics = true;
  modelify::registration_toolbox::matchDescriptorsFlannSearch<descriptor_type>(
      detection_descriptors, object_descriptors, flann_params, correspondences);
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  LOG(INFO) << "Time correspondences: "
            << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() / 1e6;
  if (correspondences->empty()) {
    LOG(ERROR) << "No correspondences found!";
    return Transformation();
  }
  LOG(INFO) << "Found " << correspondences->size() << " correspondences.";

  // Convert keypoints into matrix
  begin = std::chrono::steady_clock::now();
  Eigen::Matrix<double, 3, Eigen::Dynamic> object_matrix(3, correspondences->size());
  Eigen::Matrix<double, 3, Eigen::Dynamic> detection_matrix(3, correspondences->size());
  uint idx = 0;
  for (const auto& correspondence : *correspondences) {
    object_matrix.col(idx) << object_keypoints->points[correspondence.index_match].x,
        detection_keypoints->points[correspondence.index_match].y,
        detection_keypoints->points[correspondence.index_match].z;
    detection_matrix.col(idx) << detection_keypoints->points[correspondence.index_query].x,
        detection_keypoints->points[correspondence.index_query].y,
        detection_keypoints->points[correspondence.index_query].z;
    ++idx;
  }
  end = std::chrono::steady_clock::now();
  LOG(INFO) << "Time conversion: "
            << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() / 1e6;

  // Solve with TEASER++
  teaser::RobustRegistrationSolver::Params params;
  params.estimate_scaling = false;
  params.rotation_cost_threshold = 0.005;
  teaser::RobustRegistrationSolver solver(params);
  begin = std::chrono::steady_clock::now();
  teaser::RegistrationSolution solution = solver.solve(detection_matrix, object_matrix);
  end = std::chrono::steady_clock::now();
  LOG(INFO) << "Time teaser: "
            << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() / 1e6;

  if (!solution.valid) {
    LOG(ERROR) << "Registration using Teaser failed!";
    return Transformation();
  }

  Eigen::Matrix3f rotation_matrix = solution.rotation.cast<float>();
  Transformation T_teaser =
      Transformation(solution.translation.cast<float>(), Quaternion(rotation_matrix));
  LOG(INFO) << "Transformation teaser:\n" << T_teaser.getTransformationMatrix();
  return T_teaser;
}

ObjectDetector3D::Transformation ObjectDetector3D::computeTransformFromCorrespondences(
    const modelify::PointSurfelCloudType::Ptr& detection_keypoints,
    const modelify::PointSurfelCloudType::Ptr& object_keypoints,
    const modelify::CorrespondencesTypePtr& correspondences) {
  // Filter correspondences
  modelify::registration_toolbox::RansacParams ransac_params;
  modelify::CorrespondencesTypePtr filtered_correspondences(new modelify::CorrespondencesType());
  if (!modelify::registration_toolbox::filterCorrespondences(detection_keypoints, object_keypoints,
                                                             ransac_params, correspondences,
                                                             filtered_correspondences)) {
    // TODO(gasserl): error instead of warning?
    LOG(WARNING) << "Filtering correspondences failed!";
    *filtered_correspondences = *correspondences;
  }
  LOG(INFO) << "Filtered correspondences to " << filtered_correspondences->size();

  // Align features
  modelify::registration_toolbox::GeometricConsistencyParams consistency_params;
  consistency_params.consensus_set_resolution_m = 0.05;
  modelify::TransformationVector T_geometric_consistency;
  std::vector<modelify::CorrespondencesType> clustered_correspondences;
  if (!modelify::registration_toolbox::alignKeypointsGeometricConsistency(
          detection_keypoints, object_keypoints, filtered_correspondences,
          consistency_params.min_cluster_size, consistency_params.consensus_set_resolution_m,
          &T_geometric_consistency, &clustered_correspondences)) {
    LOG(ERROR) << "Keypoint alignment failed!";
    return Transformation();
  }
  *correspondences = clustered_correspondences[0];
  LOG(INFO) << "Aligned keypoints, best alignment found with "
            << clustered_correspondences[0].size() << " correspondences and transform\n"
            << T_geometric_consistency[0];

  if (!Quaternion::isValidRotationMatrix(T_geometric_consistency[0].block<3, 3>(0, 0))) {
    for (int i = 0; i < 3; ++i) {
      T_geometric_consistency[0].block<3, 1>(0, i) =
          T_geometric_consistency[0].block<3, 1>(0, i).normalized();
    }
    LOG(WARNING) << "Normalized rotation matrix, new transformation:\n"
                 << T_geometric_consistency[0];
  }
  return Transformation(T_geometric_consistency[0]);
}

ObjectDetector3D::Transformation ObjectDetector3D::icpUsingModelify(
    const modelify::PointSurfelCloudType::Ptr& detection_surfels,
    const modelify::PointSurfelCloudType::Ptr& object_surfels,
    const Transformation& transform_init) {
  // Validate initial alignment
  modelify::registration_toolbox::ICPParams icp_params;
  double cloud_resolution = modelify::kInvalidCloudResolution;
  double mean_squared_distance;
  double inlier_ratio;
  std::vector<size_t> outlier_indices;
  modelify::registration_toolbox::validateAlignment<modelify::PointSurfelType>(
      detection_surfels, object_surfels, transform_init.getTransformationMatrix(), icp_params,
      cloud_resolution, &mean_squared_distance, &inlier_ratio, &outlier_indices);
  LOG(INFO) << "Initial validation results: " << mean_squared_distance << " mean squared distance, "
            << inlier_ratio << " inlier ratio";

  // Refine transformation with modelify ICP
  modelify::Transformation T_icp;
  double mean_squared_distance_icp = 0;
  double inlier_ratio_icp = mean_squared_distance;
  if (!estimateTransformationPointToPoint(detection_surfels, object_surfels,
                                          transform_init.getTransformationMatrix(), icp_params,
                                          cloud_resolution, &T_icp, &mean_squared_distance_icp)) {
    LOG(WARNING) << "Keypoint ICP refinement failed!";
  } else {
    if (!Quaternion::isValidRotationMatrix(T_icp.block<3, 3>(0, 0))) {
      for (int i = 0; i < 3; ++i) {
        T_icp.block<3, 1>(0, i) = T_icp.block<3, 1>(0, i).normalized();
      }
    }
    Transformation transform_icp = Transformation(T_icp) * transform_init;

    // Validate alignment ICP
    modelify::registration_toolbox::validateAlignment<modelify::PointSurfelType>(
        detection_surfels, object_surfels, transform_icp.getTransformationMatrix(), icp_params,
        cloud_resolution, &mean_squared_distance_icp, &inlier_ratio_icp, &outlier_indices);
    LOG(INFO) << "ICP validation results: " << mean_squared_distance_icp
              << " mean squared distance, " << inlier_ratio_icp << " inlier ratio";

    if (inlier_ratio_icp >= inlier_ratio) {
      // && mean_squared_distance_icp < mean_squared_distance) {
      return transform_icp;
    } else {
      LOG(WARNING) << "ICP didn't improve alignment!";
    }
  }
  return transform_init;
}

template <typename descriptor_type>
bool ObjectDetector3D::get3dFeatures(
    const pcl::PointCloud<pcl::PointXYZ>& pointcloud_xyz,
    const modelify::PointSurfelCloudType::Ptr& pointcloud_surfel_ptr,
    const modelify::PointSurfelCloudType::Ptr& keypoints,
    const typename pcl::PointCloud<descriptor_type>::Ptr& descriptors) {
  ros::WallTime start = ros::WallTime::now();
  // Get surfels
  pcl::NormalEstimation<pcl::PointXYZ, modelify::PointSurfelType> normal_estimator;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr pcl_ptr =
      boost::make_shared<const pcl::PointCloud<pcl::PointXYZ>>(pointcloud_xyz);
  normal_estimator.setInputCloud(pcl_ptr);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZ>());
  normal_estimator.setSearchMethod(kd_tree);
  // TODO(gasserl): smarter param?
  //  constexpr double search_radius = 0.1;
  //  normal_estimator.setRadiusSearch(search_radius);
  constexpr int k_radius = 4;
  normal_estimator.setKSearch(k_radius);
  normal_estimator.compute(*pointcloud_surfel_ptr);
  for (size_t i = 0; i < pointcloud_xyz.size(); ++i) {
    pointcloud_surfel_ptr->points[i].x = pointcloud_xyz.points[i].x;
    pointcloud_surfel_ptr->points[i].y = pointcloud_xyz.points[i].y;
    pointcloud_surfel_ptr->points[i].z = pointcloud_xyz.points[i].z;
  }

  size_t size_before = pointcloud_surfel_ptr->size();
  size_t i = 0;
  while (i < pointcloud_surfel_ptr->size()) {
    if (std::isnan(pointcloud_surfel_ptr->points[i].normal_x) ||
        std::isnan(pointcloud_surfel_ptr->points[i].normal_y) ||
        std::isnan(pointcloud_surfel_ptr->points[i].normal_z)) {
      pointcloud_surfel_ptr->points.erase(pointcloud_surfel_ptr->points.begin() + i);
    } else if (std::isnan(pointcloud_surfel_ptr->points[i].x) ||
               std::isnan(pointcloud_surfel_ptr->points[i].y) ||
               std::isnan(pointcloud_surfel_ptr->points[i].z)) {
      pointcloud_surfel_ptr->points.erase(pointcloud_surfel_ptr->points.begin() + i);
    } else {
      ++i;
    }
  }
  if (size_before > pointcloud_surfel_ptr->size()) {
    LOG(INFO) << "Filtered " << size_before - pointcloud_surfel_ptr->size()
              << " points with NaN points or normals";
  }
  LOG(INFO) << "Time normals: " << (ros::WallTime::now() - start).toSec();

  ros::WallTime start_keypoints = ros::WallTime::now();
  if (!getKeypoints(keypoint_type_, pointcloud_surfel_ptr, keypoints)) {
    return false;
  }
  LOG(INFO) << "Time keypoints: " << (ros::WallTime::now() - start_keypoints).toSec();

  ros::WallTime start_descriptors = ros::WallTime::now();
  getDescriptors<descriptor_type>(pointcloud_surfel_ptr, keypoints, descriptors);
  LOG(INFO) << "Time descriptors: " << (ros::WallTime::now() - start_descriptors).toSec();
  LOG(INFO) << "Time 3D features: " << (ros::WallTime::now() - start).toSec();
  return true;
}

bool ObjectDetector3D::getKeypoints(
    const KeypointType& keypoint_type,
    const modelify::PointSurfelCloudType::Ptr& pointcloud_surfel_ptr,
    const modelify::PointSurfelCloudType::Ptr& keypoints) {
  switch (keypoint_type) {
    case kIss:
      return getIssKeypoints(pointcloud_surfel_ptr, keypoints);
    case kHarris:
      return getHarrisKeypoints(pointcloud_surfel_ptr, keypoints);
    case kUniform:
      return getUniformKeypoints(pointcloud_surfel_ptr, keypoints);
    default:
      LOG(ERROR) << "Unknown keypoint type! " << keypoint_type;
      LOG(INFO) << "Keypoint types:";
      for (int i = 0; i < KeypointType::kNumKeypointTypes; ++i) {
        LOG(INFO) << i << " (" << KeypointNames[i] << ")";
      }
      return false;
  }
}

bool ObjectDetector3D::getIssKeypoints(
    const modelify::PointSurfelCloudType::Ptr& pointcloud_surfel_ptr,
    const modelify::PointSurfelCloudType::Ptr& keypoints) {
  modelify::feature_toolbox::IssParams iss_params;
  if (!modelify::feature_toolbox::detectKeypointsISS(pointcloud_surfel_ptr, iss_params,
                                                     keypoints)) {
    LOG(WARNING) << "Could not extract ISS keypoints!";
    return false;
  }
  LOG(INFO) << "Extracted " << keypoints->size() << " ISS keypoints";
  return true;
}

bool ObjectDetector3D::getHarrisKeypoints(
    const modelify::PointSurfelCloudType::Ptr& pointcloud_surfel_ptr,
    const modelify::PointSurfelCloudType::Ptr& keypoints) {
  // Get keypoints
  modelify::feature_toolbox::Harris3dParams harris_params;
  if (!modelify::feature_toolbox::extractHarrisKeypoints(harris_params, pointcloud_surfel_ptr,
                                                         keypoints)) {
    LOG(WARNING) << "Could not extract Harris keypoints!";
    return false;
  }
  LOG(INFO) << "Extracted " << keypoints->size() << " Harris keypoints";

  /*LOG(INFO) << "Harris params:" << harris_params.toString();
  size_t num = 0;
  float curvature = 0;
  float min_curvature = 1e9;
  float max_curvature = 0;
  for (const auto& point : *pointcloud_surfel_ptr) {
    ++num;
    curvature += point.curvature;
    min_curvature = std::min(min_curvature, point.curvature);
    max_curvature = std::max(max_curvature, point.curvature);
  }
  LOG(INFO) << "Avg curvature: " << curvature / float(num);
  LOG(INFO) << "Min curvature: " << min_curvature;
  LOG(INFO) << "Max curvature: " << max_curvature;*/

  return true;
}

bool ObjectDetector3D::getUniformKeypoints(
    const modelify::PointSurfelCloudType::Ptr& pointcloud_surfel_ptr,
    const modelify::PointSurfelCloudType::Ptr& keypoints) {
  // Get keypoints
  modelify::feature_toolbox::UniformDownsamplingParams uniform_params;
  if (!modelify::feature_toolbox::getKeypointsFromUniformDownsampling(pointcloud_surfel_ptr,
                                                                      uniform_params, keypoints)) {
    LOG(WARNING) << "Could not extract uniform keypoints!";
    return false;
  }
  LOG(INFO) << "Extracted " << keypoints->size() << " uniform keypoints";
  return true;
}

template <>
void ObjectDetector3D::getDescriptors<modelify::DescriptorSHOT>(
    const modelify::PointSurfelCloudType::Ptr& pointcloud_surfel_ptr,
    const modelify::PointSurfelCloudType::Ptr& keypoints,
    const pcl::PointCloud<modelify::DescriptorSHOT>::Ptr& descriptors) {
  modelify::feature_toolbox::SHOTParams shot_params;
  modelify::feature_toolbox::describeKeypoints<modelify::DescriptorSHOT>(
      pointcloud_surfel_ptr, modelify::kInvalidCloudResolution, shot_params, keypoints,
      descriptors);
}

template <>
void ObjectDetector3D::getDescriptors<modelify::DescriptorFPFH>(
    const modelify::PointSurfelCloudType::Ptr& pointcloud_surfel_ptr,
    const modelify::PointSurfelCloudType::Ptr& keypoints,
    const pcl::PointCloud<modelify::DescriptorFPFH>::Ptr& descriptors) {
  modelify::feature_toolbox::FPFHParams fpfh_params;
  modelify::feature_toolbox::describeKeypoints<modelify::DescriptorFPFH>(
      pointcloud_surfel_ptr, modelify::kInvalidCloudResolution, fpfh_params, keypoints,
      descriptors);
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

void ObjectDetector3D::visualizeKeypoints(const modelify::PointSurfelCloudType::Ptr& keypoints,
                                          const ros::Time& timestamp, const std::string& frame_id,
                                          const ros::Publisher& publisher) {
  sensor_msgs::PointCloud2 keypoint_msg;
  pcl::toROSMsg(*keypoints, keypoint_msg);
  keypoint_msg.header.stamp = timestamp;
  keypoint_msg.header.frame_id = frame_id;
  publisher.publish(keypoint_msg);
}

void ObjectDetector3D::visualizeCorrespondences(
    const modelify::PointSurfelCloudType::Ptr& detection_keypoints,
    const modelify::CorrespondencesTypePtr& correspondences, const std::string& frame_id,
    const ros::Publisher& publisher) {
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = "correspondences";
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.scale.x = 0.001;
  marker.color.r = 0;
  marker.color.g = 1;
  marker.color.b = 1;
  marker.color.a = 0.5f;
  geometry_msgs::Point point_msg;
  marker.points.clear();
  for (auto it = correspondences->begin(); it != correspondences->end(); it++) {
    const auto& point_object = object_keypoints_->points[it->index_match];
    point_msg.x = point_object.x;
    point_msg.y = point_object.y;
    point_msg.z = point_object.z;
    marker.points.push_back(point_msg);
    const auto& point_detection = detection_keypoints->points[it->index_query];
    point_msg.x = point_detection.x;
    point_msg.y = point_detection.y;
    point_msg.z = point_detection.z;
    marker.points.push_back(point_msg);
  }
  marker_array.markers.push_back(marker);
  publisher.publish(marker_array);
}

void ObjectDetector3D::visualizeNormals(const modelify::PointSurfelCloudType::Ptr& surfels,
                                        const std::string& marker_namespace,
                                        const std::string& frame_id,
                                        const ros::Publisher& publisher) {
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = marker_namespace;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.scale.x = 0.001;
  marker.color.r = 1;
  marker.color.g = 1;
  marker.color.b = 1;
  marker.color.a = 0.7f;
  geometry_msgs::Point point_msg;
  marker.points.clear();
  for (const auto& point : surfels->points) {
    point_msg.x = point.x;
    point_msg.y = point.y;
    point_msg.z = point.z;
    marker.points.push_back(point_msg);
    double length = std::sqrt(std::pow(point.normal_x, 2.0) + std::pow(point.normal_y, 2.0) +
                              std::pow(point.normal_z, 2.0)) /
                    0.01;
    point_msg.x = point.x + point.normal_x / length;
    point_msg.y = point.y + point.normal_y / length;
    point_msg.z = point.z + point.normal_z / length;
    marker.points.push_back(point_msg);
  }
  marker_array.markers.push_back(marker);
  publisher.publish(marker_array);
}

}  // namespace object_detection
}  // namespace cad_percept