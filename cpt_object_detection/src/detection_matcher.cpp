#include "cpt_object_detection/detection_matcher.h"

#include <geometry_msgs/TransformStamped.h>
#include <cgal_msgs/TriangleMeshStamped.h>
#include <cgal_conversions/mesh_conversions.h>
#include <tf/transform_broadcaster.h>
#include <pointmatcher_ros/point_cloud.h>
#include <pcl/common/pca.h>
#include <cpt_utils/pc_processing.h>
#include <minkindr_conversions/kindr_msg.h>

#include <modelify/pcl_common.h>
#include <modelify/feature_toolbox/keypoint_toolbox_3d.h>
#include <modelify/feature_toolbox/descriptor_toolbox_3d.h>
#include <modelify/registration_toolbox/registration_toolbox.h>
#include <modelify/registration_toolbox/fast_global_registration.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace cad_percept {
namespace object_detection {

ObjectDetector3D::ObjectDetector3D(const ros::NodeHandle& nh,
                                   const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      detection_frame_id_("camera_depth_optical_frame"),
      keypoint_type_(kHarris),
      descriptor_type_(kShot),
      matching_method_(kConventional),
      pointcloud_topic_("/camera/depth/color/points"),
      object_frame_id_("object_detection_mesh"),
      num_points_icp_(500),
      downsampling_(true) {
  LOG(INFO) << "[ObjectDetector3D] Object mesh with "
            << mesh_model_->getMesh().size_of_facets()
            << " facets and " << mesh_model_->getMesh().size_of_vertices()
            << " vertices";

  getParamsFromRos();
  subscribeToTopics();
  advertiseTopics();

  const std::string& off_file =
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
  nh_private_.param("downsampling", downsampling_, downsampling_);

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

  object_keypoint_pub_ =
      nh_private_.advertise<sensor_msgs::PointCloud2>("object_keypoints", 1,
                                                      true);
  LOG(INFO) << "[ObjectDetector3D] Publishing object_keypoints to topic ["
            << object_keypoint_pub_.getTopic() << "]";
  detection_keypoint_pub_ =
      nh_private_.advertise<sensor_msgs::PointCloud2>("detection_keypoints", 1,
                                                      true);
  LOG(INFO) << "[ObjectDetector3D] Publishing detection_keypoints to topic ["
            << detection_keypoint_pub_.getTopic() << "]";

  correspondences_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>(
          "correspondences", 1, true);
  normals_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>(
          "surface_normals", 1, true);
}

void ObjectDetector3D::processObject() {
  // Sample a pointcloud from the object mesh
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

  // Get 3D features of object pointcloud
  object_surfels_.reset(new modelify::PointSurfelCloudType());
  object_keypoints_.reset(new modelify::PointSurfelCloudType());
  switch (descriptor_type_) {
    case kFpfh :
      object_descriptors_fpfh_.reset(new modelify::DescriptorFPFHCloudType());
      get3dFeatures<modelify::DescriptorFPFH>(object_pointcloud_,
                                              object_surfels_,
                                              object_keypoints_,
                                              object_descriptors_fpfh_);
      break;
    case kShot :
      object_descriptors_shot_.reset(new modelify::DescriptorSHOTCloudType());
      get3dFeatures<modelify::DescriptorSHOT>(object_pointcloud_,
                                              object_surfels_,
                                              object_keypoints_,
                                              object_descriptors_shot_);
      break;
    default :
      LOG(ERROR) << "Unknown descriptor type! " << descriptor_type_;
      LOG(INFO) << "Descriptor types:";
      for (int i = 0; i < DescriptorType::kNumDescriptorTypes; ++i) {
        LOG(INFO) << i << " (" << DescriptorNames[i] << ")";
      }
      break;
  }
  LOG(INFO) << "Obtained 3D features of object pointcloud";

  // Serialize pointcloud to message
  pcl::toROSMsg(*object_surfels_, object_pointcloud_msg_);

  // Visualize object
  bool visualize_object_on_startup = false;
  nh_private_.param("visualize_object_on_startup",
                    visualize_object_on_startup,
                    visualize_object_on_startup);
  if (visualize_object_on_startup) {
    visualizeObjectPointcloud(ros::Time::now(), detection_frame_id_);
    visualizeObjectMesh(ros::Time::now(), detection_frame_id_,
                        object_mesh_init_pub_);
    visualizeKeypoints(object_keypoints_, ros::Time::now(), detection_frame_id_,
                       object_keypoint_pub_);
    LOG(INFO) << "[ObjectDetector3D] Visualizing object";
  }
}

void ObjectDetector3D::objectDetectionCallback(
    const sensor_msgs::PointCloud2 &cloud_msg_in) {
  detection_frame_id_ = cloud_msg_in.header.frame_id;
  detection_pointcloud_msg_ = cloud_msg_in;
  pcl::fromROSMsg(detection_pointcloud_msg_, detection_pointcloud_);

//  processDetectionUsingPcaAndIcp();
  processPointcloudUsing3dFeatures();
}

// TODO(gasserl): make child classes for each thing?
void ObjectDetector3D::processDetectionUsingPcaAndIcp() {
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
  visualizeObjectMesh(detection_pointcloud_msg_.header.stamp,
                      object_frame_id_ + "_init", object_mesh_init_pub_);

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
  visualizeObjectMesh(detection_pointcloud_msg_.header.stamp,
                      object_frame_id_, object_mesh_pub_);
  LOG(INFO) << "Total matching time: "
            << (ros::WallTime::now() - time_start).toSec() << " s";
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
    LOG(INFO) << "R*R^T - I:\n"
              << rotation_matrix * rotation_matrix.transpose()
                     - Eigen::Matrix3f::Identity();
    return false;
  }

  *T_object_detection_init = Transformation(rotation, translation).inverse();
  LOG(INFO) << "Time PCA: "
            << (ros::WallTime::now() - time_start).toSec() << " s";
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

  LOG(INFO) << "Time ICP: "
            << (ros::WallTime::now() - time_start).toSec() << " s";
  LOG(INFO) << "ICP on detection pointcloud "
               "and object mesh vertices successful!";
  return true;
}

// TODO(gasserl): another child class?
void ObjectDetector3D::processPointcloudUsing3dFeatures() {
  // Downsampling detection pointcloud
  if (downsampling_) {
    pcl::PointCloud<pcl::PointXYZ> temp(detection_pointcloud_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr detection_pointcloud_ptr =
        boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(temp);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
    voxel_grid_filter.setInputCloud(detection_pointcloud_ptr);
    constexpr float voxel_size = 0.002;
    voxel_grid_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel_grid_filter.filter(detection_pointcloud_);
    LOG(INFO) << "Detection pointcloud downsampled to resolution of "
              << voxel_size << " m, resulting in "
              << detection_pointcloud_.size() << " points";
  }

  // Compute transform between detection and object
  Transformation T_object_detection;
  bool success;
  switch (descriptor_type_) {
    case kFpfh :
      success = computeTransformUsing3dFeatures<modelify::DescriptorFPFH>(
          object_descriptors_fpfh_, &T_object_detection);
      break;
    case kShot:
      success = computeTransformUsing3dFeatures<modelify::DescriptorSHOT>(
          object_descriptors_shot_, &T_object_detection);
      break;
    default :
      LOG(ERROR) << "Unknown descriptor type! " << descriptor_type_;
      LOG(INFO) << "Descriptor types:";
      for (int i = 0; i < DescriptorType::kNumDescriptorTypes; ++i) {
        LOG(INFO) << i << " (" << DescriptorNames[i] << ")";
      }
      success = false;
      break;
  }

  if (!success) {
    LOG(ERROR) << "Matching features was not successful!";
    return;
  }

  if (!T_object_detection.getTransformationMatrix().allFinite()) {
    LOG(ERROR) << "Transformation not valid!";
    LOG(INFO) << "Transformation:\n" << T_object_detection;
    return;
  }

  // Publish results
  LOG(INFO) << "Publishing results";
  publishTransformation(T_object_detection.inverse(),
                        detection_pointcloud_msg_.header.stamp,
                        detection_frame_id_, object_frame_id_);
  visualizeObjectPointcloud(detection_pointcloud_msg_.header.stamp,
                            detection_frame_id_);
  visualizeObjectMesh(detection_pointcloud_msg_.header.stamp,
                      object_frame_id_, object_mesh_pub_);
}

template <typename descriptor_type>
bool ObjectDetector3D::computeTransformUsing3dFeatures(
    const typename pcl::PointCloud<descriptor_type>::Ptr& object_descriptors,
    Transformation* T_object_detection) {
  if (object_descriptors->empty()) {
    LOG(ERROR) << "Object pointcloud has no features!";
    return false;
  }

  // Find features of detection pointcloud
  modelify::PointSurfelCloudType::Ptr detection_surfels(
      new modelify::PointSurfelCloudType());
  modelify::PointSurfelCloudType::Ptr detection_keypoints(
      new modelify::PointSurfelCloudType());
  typename pcl::PointCloud<descriptor_type>::Ptr detection_descriptors(
      new pcl::PointCloud<descriptor_type>());
  get3dFeatures<descriptor_type>(detection_pointcloud_, detection_surfels,
                                 detection_keypoints, detection_descriptors);

  // Visualizations
  visualizeNormals(object_surfels_, "object",
                   detection_pointcloud_msg_.header.frame_id,
                   normals_pub_);
  visualizeNormals(detection_surfels, "detection",
                   detection_pointcloud_msg_.header.frame_id,
                   normals_pub_);
  visualizeKeypoints(detection_keypoints,
                     detection_pointcloud_msg_.header.stamp,
                     detection_pointcloud_msg_.header.frame_id,
                     detection_keypoint_pub_);
  visualizeKeypoints(object_keypoints_,
                     detection_pointcloud_msg_.header.stamp,
                     detection_pointcloud_msg_.header.frame_id,
                     object_keypoint_pub_);

  if (detection_descriptors->empty()) {
    LOG(ERROR) << "Detection pointcloud has no features!";
    return false;
  }

  // Match features
  switch (matching_method_) {
    case kConventional:
      return computeTransformUsingModelify<descriptor_type>(
          object_descriptors, detection_surfels, detection_keypoints,
          detection_descriptors, T_object_detection);
    case kFastGlobalRegistration:
      return computeTransformUsingFgr<descriptor_type>(
          object_descriptors, detection_surfels, detection_keypoints,
          detection_descriptors, T_object_detection);
    default :
      LOG(ERROR) << "Unknown matching method! " << matching_method_;
      LOG(INFO) << "Matching methods:";
      for (int i = 0; i < MatchingMethod::kNumMatchingMethods; ++i) {
        LOG(INFO) << i << " (" << MatchingMethodNames[i] << ")";
      }
      return false;
  }
}

template <typename descriptor_type>
bool ObjectDetector3D::computeTransformUsingFgr(
    const typename pcl::PointCloud<descriptor_type>::Ptr& object_descriptors,
    const modelify::PointSurfelCloudType::Ptr& detection_surfels,
    const modelify::PointSurfelCloudType::Ptr& detection_keypoints,
    const typename pcl::PointCloud<descriptor_type>::Ptr& detection_descriptors,
    Transformation* T_object_detection) {
  modelify::registration_toolbox::FastGlobalRegistrationParams fgr_params;
  fgr_params.crosscheck_test = true;  // 300 -> 30
  fgr_params.tuple_test = false;      // 30 -> 0
  fgr_params.refine_using_icp = true;
  fgr_params.use_absolute_scale = true;
  modelify::Correspondences correspondences;
  modelify::Transformation transform;
  if (!modelify::registration_toolbox::fast_global_registration::
          estimateTransformationFastGlobalRegistration<
              modelify::PointSurfelType, descriptor_type>(
                detection_surfels, object_surfels_, detection_keypoints,
                object_keypoints_, detection_descriptors, object_descriptors,
                fgr_params, &correspondences, &transform)) {
    LOG(ERROR) << "Fast global registration was not successful!";
    return false;
  }
  modelify::CorrespondencesTypePtr corr_ptr(new modelify::CorrespondencesType());
  for (const modelify::CorrespondencePair& correspondence : correspondences) {
    pcl::Correspondence corr;
    corr.index_match = correspondence.first;
    corr.index_query = correspondence.second;
    corr_ptr->push_back(corr);
  }
  visualizeCorrespondences(detection_keypoints, corr_ptr,
                           detection_frame_id_, correspondences_pub_);
  *T_object_detection = Transformation(transform);
  return true;
}

template <typename descriptor_type>
bool ObjectDetector3D::computeTransformUsingModelify(
    const typename pcl::PointCloud<descriptor_type>::Ptr& object_descriptors,
    const modelify::PointSurfelCloudType::Ptr& detection_surfels,
    const modelify::PointSurfelCloudType::Ptr& detection_keypoints,
    const typename pcl::PointCloud<descriptor_type>::Ptr& detection_descriptors,
    Transformation* T_object_detection) {
  // Find correspondences
  modelify::registration_toolbox::FlannSearchMatchingParams flann_params;
  if (object_descriptors->size() < flann_params.num_of_correspondences ||
      detection_descriptors->size() < flann_params.num_of_correspondences) {
    LOG(ERROR) << "Too few features found! Object: "
               << object_descriptors->size() << "/"
               << flann_params.num_of_correspondences << ", Detection: "
               << detection_descriptors->size() << "/"
               << flann_params.num_of_correspondences;
    return false;
  }
  modelify::CorrespondencesTypePtr correspondences(
      new modelify::CorrespondencesType());
  modelify::registration_toolbox::matchDescriptorsFlannSearch<descriptor_type>(
      detection_descriptors, object_descriptors, flann_params, correspondences);

  if (correspondences->empty()) {
    LOG(ERROR) << "No correspondences found!";
    return false;
  }
  LOG(INFO) << "Found " << correspondences->size() << " correspondences.";

  visualizeCorrespondences(detection_keypoints, correspondences,
                           detection_pointcloud_msg_.header.frame_id,
                           correspondences_pub_);

  // Get transformation between detection and object pointcloud
  return computeTransformFromCorrespondences(detection_surfels, detection_keypoints,
                                             correspondences, T_object_detection);
}

template <typename descriptor_type>
bool ObjectDetector3D::get3dFeatures(
    const pcl::PointCloud<pcl::PointXYZ>& pointcloud_xyz,
    const modelify::PointSurfelCloudType::Ptr& pointcloud_surfel_ptr,
    const modelify::PointSurfelCloudType::Ptr& keypoints,
    const typename pcl::PointCloud<descriptor_type>::Ptr& descriptors) {
  // Get surfels
  pcl::NormalEstimation<pcl::PointXYZ,
                        modelify::PointSurfelType> normal_estimator;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr pcl_ptr =
      boost::make_shared<const pcl::PointCloud<pcl::PointXYZ>>(pointcloud_xyz);
  normal_estimator.setInputCloud(pcl_ptr);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree(
      new pcl::search::KdTree<pcl::PointXYZ>());
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
      pointcloud_surfel_ptr->points.erase(
          pointcloud_surfel_ptr->points.begin() + i);
    } else if (std::isnan(pointcloud_surfel_ptr->points[i].x) ||
        std::isnan(pointcloud_surfel_ptr->points[i].y) ||
        std::isnan(pointcloud_surfel_ptr->points[i].z)) {
      pointcloud_surfel_ptr->points.erase(
          pointcloud_surfel_ptr->points.begin() + i);
    } else {
      ++i;
    }
  }
  if (size_before > pointcloud_surfel_ptr->size()) {
    LOG(INFO) << "Filtered " << size_before - pointcloud_surfel_ptr->size()
              << " points with NaN points or normals";
  }

  LOG(INFO) << "Computed " << pointcloud_surfel_ptr->size() << " surfel points";

  if (!getKeypoints(keypoint_type_, pointcloud_surfel_ptr, keypoints)) {
    return false;
  }

  getDescriptors<descriptor_type>(pointcloud_surfel_ptr, keypoints, descriptors);
  return true;
}

bool ObjectDetector3D::getKeypoints(
    const KeypointType& keypoint_type,
    const modelify::PointSurfelCloudType::Ptr& pointcloud_surfel_ptr,
    const modelify::PointSurfelCloudType::Ptr& keypoints) {
  switch (keypoint_type) {
    case kIss :
      return getIssKeypoints(pointcloud_surfel_ptr, keypoints);
    case kHarris :
      return getHarrisKeypoints(pointcloud_surfel_ptr, keypoints);
    case kUniform :
      return getUniformKeypoints(pointcloud_surfel_ptr, keypoints);
    default :
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
  if (!modelify::feature_toolbox::detectKeypointsISS(pointcloud_surfel_ptr,
                                                     iss_params,
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
  if (!modelify::feature_toolbox::extractHarrisKeypoints(
      harris_params, pointcloud_surfel_ptr, keypoints)) {
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
  if (!modelify::feature_toolbox::getKeypointsFromUniformDownsampling(
      pointcloud_surfel_ptr, uniform_params, keypoints)) {
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
      pointcloud_surfel_ptr, modelify::kInvalidCloudResolution, shot_params,
      keypoints, descriptors);
  LOG(INFO) << "Found SHOT descriptors for keypoints";
}

template <>
void ObjectDetector3D::getDescriptors<modelify::DescriptorFPFH>(
    const modelify::PointSurfelCloudType::Ptr& pointcloud_surfel_ptr,
    const modelify::PointSurfelCloudType::Ptr& keypoints,
    const pcl::PointCloud<modelify::DescriptorFPFH>::Ptr& descriptors) {
  modelify::feature_toolbox::FPFHParams fpfh_params;
  modelify::feature_toolbox::describeKeypoints<modelify::DescriptorFPFH>(
      pointcloud_surfel_ptr, modelify::kInvalidCloudResolution, fpfh_params,
      keypoints, descriptors);
  LOG(INFO) << "Found FPFH descriptors for keypoints";
}

bool ObjectDetector3D::computeTransformFromCorrespondences(
    const modelify::PointSurfelCloudType::Ptr& detection_surfels,
    const modelify::PointSurfelCloudType::Ptr& detection_keypoints,
    const modelify::CorrespondencesTypePtr& correspondences,
    Transformation* transform) {
  // Filter correspondences
  modelify::registration_toolbox::RansacParams ransac_params;
  modelify::CorrespondencesTypePtr filtered_correspondences(
      new modelify::CorrespondencesType());
  if (!modelify::registration_toolbox::filterCorrespondences(
      detection_keypoints, object_keypoints_, ransac_params, correspondences,
      filtered_correspondences)) {
    // TODO(gasserl): error instead of warning?
    LOG(WARNING) << "Filtering correspondences failed!";
    *filtered_correspondences = *correspondences;
  }

  // Align features
  modelify::registration_toolbox::GeometricConsistencyParams consistency_params;
  modelify::TransformationVector transforms;
  std::vector<modelify::CorrespondencesType> clustered_correspondences;
  if (!modelify::registration_toolbox::alignKeypointsGeometricConsistency(
      detection_keypoints, object_keypoints_, filtered_correspondences,
      consistency_params.min_cluster_size,
      consistency_params.consensus_set_resolution_m,
      &transforms, &clustered_correspondences)) {
    LOG(ERROR) << "Keypoint alignment failed!";
    return false;
  }

  LOG(INFO) << "Publishing initial transformation from geometric consistency";
  publishTransformation(Transformation(transforms[0]).inverse(),
                        detection_pointcloud_msg_.header.stamp,
                        detection_frame_id_, object_frame_id_ + "_init");
  visualizeObjectMesh(detection_pointcloud_msg_.header.stamp,
                      object_frame_id_ + "_init", object_mesh_init_pub_);

  // Validate alignment
  modelify::registration_toolbox::ICPParams icp_params;
  double cloud_resolution = 0;
  double mean_squared_distance;
  double inlier_ratio;
  std::vector<size_t> outlier_indices;
  modelify::registration_toolbox::validateAlignment<modelify::PointSurfelType>(
      detection_surfels, object_surfels_, transforms[0], icp_params,
      cloud_resolution, &mean_squared_distance, &inlier_ratio, &outlier_indices);
  LOG(INFO) << "Geometric alignment results: "
            << mean_squared_distance << " mean squared distance, "
            << inlier_ratio << " inlier ratio";
  // TODO(gasserl): use validation!?

  // Refine transformation with ICP
  modelify::Transformation transform_icp;
  if (estimateTransformationPointToPoint(
      detection_surfels, object_surfels_, transforms[0], icp_params,
      cloud_resolution, &transform_icp, &mean_squared_distance)) {
    *transform = Transformation(transform_icp);
    return true;
  }
  LOG(WARNING) << "Keypoint ICP refinement failed!";

  Transformation T_icp;
  if (performICP(Transformation(transforms[0]), &T_icp)) {
    *transform = T_icp;
    return true;
  }
  LOG(WARNING) << "Pointcloud ICP refinement failed!";

  // TODO(gasserl): return false?
  *transform = Transformation(transforms[0]);
  return true;
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
    const ros::Time& timestamp, const std::string& frame_id,
    const ros::Publisher& publisher) const {
  cgal_msgs::TriangleMeshStamped p_msg;

  // triangle mesh to prob. msg
  cgal_msgs::TriangleMesh t_msg;
  cgal::Polyhedron mesh = mesh_model_->getMesh();
  cgal::triangleMeshToMsg(mesh, &t_msg);
  p_msg.mesh = t_msg;

  p_msg.header.frame_id = frame_id;
  p_msg.header.stamp = timestamp;
  p_msg.header.seq = 0;
  publisher.publish(p_msg);
}

void ObjectDetector3D::visualizeObjectPointcloud(const ros::Time& timestamp,
                                                 const std::string& frame_id) {
  object_pointcloud_msg_.header.stamp = timestamp;
  object_pointcloud_msg_.header.frame_id = frame_id;
  object_pointcloud_pub_.publish(object_pointcloud_msg_);
}

void ObjectDetector3D::visualizeKeypoints(
    const modelify::PointSurfelCloudType::Ptr& keypoints,
    const ros::Time& timestamp,  const std::string& frame_id,
    const ros::Publisher& publisher) {
  sensor_msgs::PointCloud2 keypoint_msg;
  pcl::toROSMsg(*keypoints, keypoint_msg);
  keypoint_msg.header.stamp = timestamp;
  keypoint_msg.header.frame_id = frame_id;
  publisher.publish(keypoint_msg);
}

void ObjectDetector3D::visualizeCorrespondences(
    const modelify::PointSurfelCloudType::Ptr& detection_keypoints,
    const modelify::CorrespondencesTypePtr& correspondences,
    const std::string& frame_id, const ros::Publisher& publisher) {
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = "correspondences";
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.scale.x = 0.001;
  marker.color.r = 0;
  marker.color.g = 0.5;
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

void ObjectDetector3D::visualizeNormals(
    const modelify::PointSurfelCloudType::Ptr& surfels,
    const std::string& marker_namespace, const std::string& frame_id,
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
    double length = std::sqrt(std::pow(point.normal_x, 2.0) +
                              std::pow(point.normal_y, 2.0) +
                              std::pow(point.normal_z, 2.0)) / 0.01;
    point_msg.x = point.x + point.normal_x / length;
    point_msg.y = point.y + point.normal_y / length;
    point_msg.z = point.z + point.normal_z / length;
    marker.points.push_back(point_msg);
  }
  marker_array.markers.push_back(marker);
  publisher.publish(marker_array);
}

}
}