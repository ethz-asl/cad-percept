#include "cpt_object_detection/object_detector_3d.h"

#include <cgal_conversions/mesh_conversions.h>
#include <cgal_msgs/TriangleMeshStamped.h>
#include <cpt_object_detection/learned_descriptor.h>
#include <cpt_utils/pc_processing.h>
#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>
#include <pcl/filters/voxel_grid.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

namespace cad_percept::object_detection {

ObjectDetector3D::ObjectDetector3D(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      object_frame_id_("object_detection_mesh"),
      pointcloud_topic_("/camera/depth/color/points"),
      detection_frame_id_("camera_depth_optical_frame"),
      use_3d_features_(true),
      use_kalman_filter_(false),
      use_inlier_ratio_filter_(false),
      publish_static_transform_(true),
      reference_frame_id_("world"),
      verbose_(true),
      keypoint_type_(kIss),
      descriptor_type_(kFpfh),
      matching_method_(kGeometricConsistency),
      correspondence_threshold_(0.1),
      downsampling_resolution_(0.001),
      refine_using_icp_(true),
      use_icp_on_pointcloud_(false),
      inlier_ratio_filter_(0),
      min_inlier_ratio_(0.25) {
  getParamsFromRos();
  subscribeToTopics();
  advertiseTopics();

  if (!initializeObjectMesh()) {
    std::cerr << "Could not initialize object!";
  }
}

bool ObjectDetector3D::initializeObjectMesh() {
  // Load object mesh
  const std::string& off_file = nh_private_.param<std::string>("off_model", "fail");
  if (!cgal::MeshModel::create(off_file, &mesh_model_)) {
    LOG(FATAL) << "Could not get mesh model from off file at " << off_file << "!";
    return false;
  }
  if (verbose_) {
    LOG(INFO) << "Object mesh with " << mesh_model_->getMesh().size_of_facets() << " facets and "
              << mesh_model_->getMesh().size_of_vertices() << " vertices";
  }

  // Sample pointcloud from object mesh
  if (use_3d_features_ || use_icp_on_pointcloud_) {
    std::chrono::steady_clock::time_point start_sampling = std::chrono::steady_clock::now();
    int num_points_object_pointcloud = 1e3;
    nh_private_.param("num_points_object_pointcloud", num_points_object_pointcloud,
                      num_points_object_pointcloud);
    cpt_utils::sample_pc_from_mesh(mesh_model_->getMesh(), num_points_object_pointcloud, 0.0,
                                   &object_pointcloud_);

    if (object_pointcloud_.empty()) {
      LOG(FATAL) << "Could not sample pointcloud from object mesh!";
      return false;
    }

    if (verbose_) {
      LOG(INFO) << "Converted object mesh to a pointcloud with " << object_pointcloud_.size()
                << " points";
      LOG(INFO)
          << "Time sampling mesh: "
          << std::chrono::duration<float>(std::chrono::steady_clock::now() - start_sampling).count()
          << " s";
    }

    // Downsampling pointcloud resolution
    if (downsampling_resolution_ > 0) {
      start_sampling = std::chrono::steady_clock::now();
      pcl::PointCloud<pcl::PointXYZ>::Ptr object_pointcloud_ptr =
          boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(object_pointcloud_);
      pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
      voxel_grid_filter.setInputCloud(object_pointcloud_ptr);
      voxel_grid_filter.setLeafSize(downsampling_resolution_, downsampling_resolution_,
                                    downsampling_resolution_);
      pcl::PointCloud<pcl::PointXYZ> filtered_pointcloud;
      voxel_grid_filter.filter(filtered_pointcloud);
      object_pointcloud_ = filtered_pointcloud;
      if (verbose_) {
        LOG(INFO) << "Object pointcloud downsampled to resolution of " << downsampling_resolution_
                  << " m, resulting in " << object_pointcloud_.size() << " points";
        LOG(INFO) << "Time downsampling: "
                  << std::chrono::duration<float>(std::chrono::steady_clock::now() - start_sampling)
                         .count()
                  << " s";
      }
    }
  }

  // Get 3D features of object pointcloud
  if (use_3d_features_ || use_icp_on_pointcloud_) {
    object_surfels_ = boost::make_shared<modelify::PointSurfelCloudType>(
        estimateNormals(object_pointcloud_, *mesh_model_, verbose_));
  }
  if (use_3d_features_) {
    object_keypoints_.reset(new modelify::PointSurfelCloudType());
    bool success = false;
    switch (descriptor_type_) {
      case kFpfh:
        object_descriptors_fpfh_.reset(new modelify::DescriptorFPFHCloudType());
        success = compute3dFeatures<modelify::DescriptorFPFH>(
            keypoint_type_, object_surfels_, object_keypoints_, object_descriptors_fpfh_, verbose_);
        break;
      case kShot:
        object_descriptors_shot_.reset(new modelify::DescriptorSHOTCloudType());
        success = compute3dFeatures<modelify::DescriptorSHOT>(
            keypoint_type_, object_surfels_, object_keypoints_, object_descriptors_shot_, verbose_);
        break;
      case k3dSmoothNet:
        object_descriptors_learned_.reset(new pcl::PointCloud<LearnedDescriptor>());
        success =
            compute3dFeatures<LearnedDescriptor>(keypoint_type_, object_surfels_, object_keypoints_,
                                                 object_descriptors_learned_, verbose_);
        break;
      case kUnit:
        object_descriptors_unit_.reset(new pcl::PointCloud<UnitDescriptor>());
        success = compute3dFeatures<UnitDescriptor>(
            keypoint_type_, object_surfels_, object_keypoints_, object_descriptors_unit_, verbose_);
        break;
      default:
        LOG(ERROR) << "Unknown descriptor type! " << descriptor_type_;
        LOG(INFO) << "Descriptor types:";
        for (int i = 0; i < DescriptorType::kNumDescriptorTypes; ++i) {
          LOG(INFO) << i << " (" << DescriptorNames[i] << ")";
        }
        break;
    }

    if (!success) {
      LOG(ERROR) << "Could not compute 3D features of object pointcloud!";
      return false;
    }
    if (verbose_) {
      LOG(INFO) << "Obtained 3D features of object pointcloud";
    }
  }

  // Visualize object
  bool visualize_object_on_startup = false;
  nh_private_.param("visualize_object_on_startup", visualize_object_on_startup,
                    visualize_object_on_startup);
  if (visualize_object_on_startup) {
    visualizeMesh(mesh_model_, ros::Time::now(), detection_frame_id_, object_mesh_init_pub_);
    if (use_3d_features_) {
      visualizePointcloud(object_pointcloud_, ros::Time::now(), detection_frame_id_,
                          object_pointcloud_pub_);
      visualizeKeypoints(object_keypoints_, ros::Time::now(), detection_frame_id_,
                         object_keypoint_pub_);
    }
    if (verbose_) {
      LOG(INFO) << "Visualizing object";
    }
  }

  return true;
}

void ObjectDetector3D::getParamsFromRos() {
  nh_private_.param("detection_pointcloud_topic", pointcloud_topic_, pointcloud_topic_);
  nh_private_.param("scene_pointcloud_topic", scene_topic_, scene_topic_);
  nh_private_.param("object_frame_id", object_frame_id_, object_frame_id_);
  nh_private_.param("use_3d_features", use_3d_features_, use_3d_features_);
  nh_private_.param("use_inlier_ratio_filter", use_inlier_ratio_filter_, use_inlier_ratio_filter_);
  nh_private_.param("min_inlier_ratio", min_inlier_ratio_, min_inlier_ratio_);
  nh_private_.param("use_kalman_filter", use_kalman_filter_, use_kalman_filter_);
  nh_private_.param("publish_static_transform", publish_static_transform_,
                    publish_static_transform_);
  nh_private_.param("reference_frame_id", reference_frame_id_, reference_frame_id_);
  nh_private_.param("verbose", verbose_, verbose_);
  nh_private_.param("refine_using_icp", refine_using_icp_, refine_using_icp_);
  nh_private_.param("use_icp_on_pointcloud", use_icp_on_pointcloud_, use_icp_on_pointcloud_);
  nh_private_.param("icp_config_file", icp_config_file_, icp_config_file_);
  nh_private_.param("correspondence_threshold", correspondence_threshold_,
                    correspondence_threshold_);
  nh_private_.param("downsampling", downsampling_resolution_, downsampling_resolution_);

  if (verbose_) {
    LOG(INFO) << "Parameters:"
              << "\n - pointcloud_topic: " << pointcloud_topic_
              << "\n - scene_topic: " << scene_topic_
              << "\n - object_frame_id: " << object_frame_id_
              << "\n - use_3d_features: " << use_3d_features_
              << "\n - use_inlier_ratio_filter: " << use_inlier_ratio_filter_
              << "\n - min_inlier_ratio: " << min_inlier_ratio_
              << "\n - use_kalman_filter: " << use_kalman_filter_
              << "\n - publish_static_transform: " << publish_static_transform_
              << "\n - reference_frame_id: " << reference_frame_id_ << "\n - verbose: " << verbose_
              << "\n - refine_using_icp: " << refine_using_icp_
              << "\n - use_icp_on_pointcloud: " << use_icp_on_pointcloud_
              << "\n - icp_config_file: " << icp_config_file_
              << "\n - correspondence_threshold: " << correspondence_threshold_
              << "\n - downsampling: " << downsampling_resolution_;
  }

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
  if (verbose_) {
    LOG(INFO) << "Using keypoint type " << KeypointNames[keypoint_type_];
  }

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
  if (verbose_) {
    LOG(INFO) << "Using descriptor type " << DescriptorNames[descriptor_type_];
  }

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
  if (verbose_) {
    LOG(INFO) << "Using matching method " << MatchingMethodNames[matching_method_];
  }

  // Kalman initialization
  nh_private_.param("use_kalman_filter", use_kalman_filter_, use_kalman_filter_);

  float kf_p_translation = 1;
  float kf_p_rotation = 10;
  nh_private_.param("kf_p_translation", kf_p_translation, kf_p_translation);
  nh_private_.param("kf_p_rotation", kf_p_rotation, kf_p_rotation);
  P_kalman_.setIdentity(7, 7);
  P_kalman_.block<3, 3>(0, 0) *= kf_p_translation;
  P_kalman_.block<4, 4>(3, 3) *= kf_p_rotation;

  float kf_r_rotation = 1;
  float kf_r_translation = 0.1;
  nh_private_.param("kf_r_rotation", kf_r_rotation, kf_r_rotation);
  nh_private_.param("kf_r_translation", kf_r_translation, kf_r_translation);
  R_kalman_ = Eigen::MatrixXf::Identity(7, 7);
  R_kalman_.block<3, 3>(0, 0) *= kf_r_rotation;
  R_kalman_.block<4, 4>(3, 3) *= kf_r_rotation;

  x_kalman_.setZero(7);
  x_kalman_(4) = 1;
}

void ObjectDetector3D::subscribeToTopics() {
  int queue_size = 1;
  nh_private_.param("queue_size", queue_size, queue_size);
  detection_pointcloud_sub_ = nh_.subscribe(pointcloud_topic_, queue_size,
                                            &ObjectDetector3D::objectDetectionCallback, this);
  scene_pointcloud_sub_ =
      nh_.subscribe(scene_topic_, queue_size, &ObjectDetector3D::sceneCallback, this);
  if (verbose_) {
    LOG(INFO) << "Subscribed to pointcloud topic [" << detection_pointcloud_sub_.getTopic() << "]";
    LOG(INFO) << "Subscribed to scene topic [" << scene_pointcloud_sub_.getTopic() << "]";
  }
}

void ObjectDetector3D::advertiseTopics() {
  object_pointcloud_pub_ = nh_private_.advertise<sensor_msgs::PointCloud2>("object_pcl", 1, true);

  object_mesh_pub_ = nh_private_.advertise<cgal_msgs::TriangleMeshStamped>("object_mesh", 1, true);
  object_mesh_init_pub_ =
      nh_private_.advertise<cgal_msgs::TriangleMeshStamped>("object_mesh_init", 1, true);

  object_keypoint_pub_ =
      nh_private_.advertise<sensor_msgs::PointCloud2>("object_keypoints", 1, true);
  detection_keypoint_pub_ =
      nh_private_.advertise<sensor_msgs::PointCloud2>("detection_keypoints", 1, true);

  correspondences_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>("correspondences", 1, true);
  normals_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("surface_normals", 1, true);

  if (verbose_) {
    LOG(INFO) << "Publishing object pointcloud to topic [" << object_pointcloud_pub_.getTopic()
              << "]";
    LOG(INFO) << "Publishing object mesh to topic [" << object_mesh_pub_.getTopic() << "]";
    LOG(INFO) << "Publishing init object mesh to topic [" << object_mesh_init_pub_.getTopic()
              << "]";
    LOG(INFO) << "[ObjectDetector3D] Publishing object_keypoints to topic ["
              << object_keypoint_pub_.getTopic() << "]";
    LOG(INFO) << "[ObjectDetector3D] Publishing detection_keypoints to topic ["
              << detection_keypoint_pub_.getTopic() << "]";
  }
}

void ObjectDetector3D::sceneCallback(const sensor_msgs::PointCloud2& cloud_msg_in) {
  pcl::fromROSMsg(cloud_msg_in, scene_pointcloud_);
}

void ObjectDetector3D::objectDetectionCallback(const sensor_msgs::PointCloud2& cloud_msg_in) {
  detection_frame_id_ = cloud_msg_in.header.frame_id;
  detection_stamp_ = cloud_msg_in.header.stamp;
  pcl::fromROSMsg(cloud_msg_in, detection_pointcloud_);

  processDetection();
}

bool ObjectDetector3D::processDetection() {
  std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();

  // Check pointclouds
  if (detection_pointcloud_.empty()) {
    LOG(ERROR) << "Empty detection pointcloud!";
    return false;
  }
  if (object_pointcloud_.empty()) {
    LOG(ERROR) << "Empty object pointcloud!";
    return false;
  }
  if (scene_pointcloud_.empty()) {
    scene_pointcloud_ = detection_pointcloud_;
  }

  // Get transformation to reference frame
  Transformation T_detection_world;
  ros::Time stamp_T;
  bool valid_transform_to_reference = true;
  if (reference_frame_id_.empty()) {
    valid_transform_to_reference = false;
  } else if (!lookupTransform(reference_frame_id_, detection_frame_id_, detection_stamp_,
                              T_detection_world, stamp_T)) {
    valid_transform_to_reference = false;
    LOG(WARNING) << "Could not get transformation between sensor frame " << detection_frame_id_
                 << " and reference frame " << reference_frame_id_ << "!";
  }

  // Prepare pointclouds
  Transformation T_object_detection_init;
  modelify::PointSurfelCloudType::Ptr detection_surfels;
  if (use_3d_features_ || use_icp_on_pointcloud_) {
    modelify::PointSurfelCloudType::Ptr object_surfels =
        boost::make_shared<modelify::PointSurfelCloudType>(
            estimateNormals(object_pointcloud_, *mesh_model_, verbose_));
    detection_surfels = boost::make_shared<modelify::PointSurfelCloudType>(
        estimateNormals(detection_pointcloud_, verbose_));
  }

  // Get initial alignment
  if (use_3d_features_) {
    // Downsampling detection pointcloud
    if (downsampling_resolution_ > 0) {
      std::chrono::steady_clock::time_point start_sampling = std::chrono::steady_clock::now();
      pcl::PointCloud<pcl::PointXYZ>::Ptr detection_pointcloud_ptr =
          boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(detection_pointcloud_);
      pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
      voxel_grid_filter.setInputCloud(detection_pointcloud_ptr);
      voxel_grid_filter.setLeafSize(downsampling_resolution_, downsampling_resolution_,
                                    downsampling_resolution_);
      pcl::PointCloud<pcl::PointXYZ> filtered_pointcloud;
      voxel_grid_filter.filter(filtered_pointcloud);
      detection_pointcloud_ = filtered_pointcloud;
      if (verbose_) {
        LOG(INFO) << "Detection pointcloud downsampled to resolution of "
                  << downsampling_resolution_ << " m, resulting in " << detection_pointcloud_.size()
                  << " points";
        LOG(INFO) << "Time downsampling: "
                  << std::chrono::duration<float>(std::chrono::steady_clock::now() - start_sampling)
                         .count()
                  << " s";
      }
    }

    // Compute transform between detection and object
    modelify::PointSurfelCloudType::Ptr detection_keypoints(new modelify::PointSurfelCloudType());
    modelify::CorrespondencesTypePtr correspondences(new modelify::CorrespondencesType());
    switch (descriptor_type_) {
      case kFpfh: {
        typename pcl::PointCloud<modelify::DescriptorFPFH>::Ptr detection_descriptors_fpfh(
            new pcl::PointCloud<modelify::DescriptorFPFH>());
        compute3dFeatures<modelify::DescriptorFPFH>(keypoint_type_, detection_surfels,
                                                    detection_keypoints, detection_descriptors_fpfh,
                                                    verbose_);
        T_object_detection_init = computeTransformUsing3dFeatures<modelify::DescriptorFPFH>(
            matching_method_, detection_surfels, detection_keypoints, detection_descriptors_fpfh,
            object_surfels_, object_keypoints_, object_descriptors_fpfh_, correspondence_threshold_,
            correspondences, verbose_);
        break;
      }
      case kShot: {
        typename pcl::PointCloud<modelify::DescriptorSHOT>::Ptr detection_descriptors_shot(
            new pcl::PointCloud<modelify::DescriptorSHOT>());
        compute3dFeatures<modelify::DescriptorSHOT>(keypoint_type_, detection_surfels,
                                                    detection_keypoints, detection_descriptors_shot,
                                                    verbose_);
        T_object_detection_init = computeTransformUsing3dFeatures<modelify::DescriptorSHOT>(
            matching_method_, detection_surfels, detection_keypoints, detection_descriptors_shot,
            object_surfels_, object_keypoints_, object_descriptors_shot_, correspondence_threshold_,
            correspondences, verbose_);
        break;
      }
      case k3dSmoothNet: {
        typename pcl::PointCloud<LearnedDescriptor>::Ptr detection_descriptors_learned(
            new pcl::PointCloud<LearnedDescriptor>());
        compute3dFeatures<LearnedDescriptor>(keypoint_type_, detection_surfels, detection_keypoints,
                                             detection_descriptors_learned, verbose_);
        T_object_detection_init = computeTransformUsing3dFeatures<LearnedDescriptor>(
            matching_method_, detection_surfels, detection_keypoints, detection_descriptors_learned,
            object_surfels_, object_keypoints_, object_descriptors_learned_,
            correspondence_threshold_, correspondences, verbose_);
        break;
      }
      case kUnit: {
        typename pcl::PointCloud<UnitDescriptor>::Ptr detection_descriptors_unit(
            new pcl::PointCloud<UnitDescriptor>());
        compute3dFeatures<UnitDescriptor>(keypoint_type_, detection_surfels, detection_keypoints,
                                          detection_descriptors_unit, verbose_);
        T_object_detection_init = computeTransformUsing3dFeatures<UnitDescriptor>(
            matching_method_, detection_surfels, detection_keypoints, detection_descriptors_unit,
            object_surfels_, object_keypoints_, object_descriptors_unit_, correspondence_threshold_,
            correspondences, verbose_);
        break;
      }
      default: {
        LOG(ERROR) << "Unknown descriptor type! " << descriptor_type_;
        LOG(INFO) << "Descriptor types:";
        for (int i = 0; i < DescriptorType::kNumDescriptorTypes; ++i) {
          LOG(INFO) << i << " (" << DescriptorNames[i] << ")";
        }
        return false;
      }

        T_object_detection_init = T_object_detection_init.inverse();
    }

    // Visualizations
    visualizeNormals(detection_surfels, "detection", detection_frame_id_, normals_pub_);
    visualizeNormals(object_surfels_, "object", detection_frame_id_, normals_pub_);
    visualizeKeypoints(detection_keypoints, detection_stamp_, detection_frame_id_,
                       detection_keypoint_pub_);
    visualizeKeypoints(object_keypoints_, detection_stamp_, detection_frame_id_,
                       object_keypoint_pub_);

    // Visualize correspondences
    visualizeCorrespondences(detection_keypoints, object_keypoints_, correspondences,
                             detection_frame_id_, correspondences_pub_);
  } else {
    // Get initial guess with PCA
    T_object_detection_init = pca(mesh_model_, detection_pointcloud_, verbose_);

    // Optimize PCA orientation
    try {
      if (use_icp_on_pointcloud_) {
        // Add normals to pointcloud
        T_object_detection_init = optimizeTransformation(object_surfels_, detection_surfels,
                                                         T_object_detection_init, verbose_);
      } else {
        T_object_detection_init = optimizeTransformation(*mesh_model_, detection_pointcloud_,
                                                         T_object_detection_init, verbose_);
      }
    } catch (...) {
      LOG(WARNING) << "Alignment optimization failed!";
    }
  }

  // Publish initial alignment
  publishTransformation(T_object_detection_init, detection_stamp_, detection_frame_id_,
                        object_frame_id_ + "_init");
  visualizeMesh(mesh_model_, detection_stamp_, object_frame_id_ + "_init", object_mesh_init_pub_);

  // Pseudo update kalman filter
  if (use_kalman_filter_ && valid_transform_to_reference) {
    // Get "measured" state
    Transformation T_kalman(T_detection_world * T_object_detection_init);
    Eigen::VectorXf z_kalman(7);
    z_kalman << T_kalman.getPosition().x(), T_kalman.getPosition().y(), T_kalman.getPosition().z(),
        T_kalman.getRotation().w(), T_kalman.getRotation().x(), T_kalman.getRotation().y(),
        T_kalman.getRotation().z();

    // Perform temporary Kalman update
    Eigen::MatrixXf K(7, 7);
    K = P_kalman_ * (P_kalman_ + R_kalman_).inverse();
    Eigen::VectorXf x(7);
    x = x_kalman_ + K * (z_kalman - x_kalman_);
    if (verbose_) {
      LOG(INFO) << "Temporary Kalman update:";
      LOG(INFO) << "Observation z:\n" << z_kalman.transpose();
      LOG(INFO) << "State x before:\n" << x_kalman_.transpose();
      LOG(INFO) << "State x after:\n" << x.transpose();
    }

    T_kalman.getPosition().x() = x_kalman_(0);
    T_kalman.getPosition().y() = x_kalman_(1);
    T_kalman.getPosition().z() = x_kalman_(2);
    Eigen::VectorXf quaternion(4);
    quaternion << x_kalman_(3), x_kalman_(4), x_kalman_(5), x_kalman_(6);
    quaternion.normalize();
    T_kalman.getRotation().setValues(quaternion(0), quaternion(1), quaternion(2), quaternion(3));

    // Transform back to local frame
    T_object_detection_init = T_detection_world.inverse() * T_kalman;
  }

  // Compare inlier ratio
  if (use_inlier_ratio_filter_) {
    double inlier_ratio_pca =
        computeInlierRatio(use_icp_on_pointcloud_, object_surfels_, detection_surfels, *mesh_model_,
                           detection_pointcloud_, T_object_detection_init);
    inlier_ratio_filter_ =
        computeInlierRatio(use_icp_on_pointcloud_, object_surfels_, detection_surfels, *mesh_model_,
                           detection_pointcloud_, T_object_detection_filter_);
    if (verbose_) {
      if (use_3d_features_) {
        LOG(INFO) << "Inlier ratio features: " << inlier_ratio_pca;
        LOG(INFO) << "Inlier ratio filter:   " << inlier_ratio_filter_;
      } else {
        LOG(INFO) << "Inlier ratio PCA:    " << inlier_ratio_pca;
        LOG(INFO) << "Inlier ratio filter: " << inlier_ratio_filter_;
      }
    }
    if (inlier_ratio_filter_ > inlier_ratio_pca) {
      T_object_detection_init = T_object_detection_filter_;
    }
  }

  // Get final alignment
  Transformation T_object_detection(T_object_detection_init);
  if (refine_using_icp_) {
    if (use_3d_features_) {
      // Refine using ICP
      modelify::PointSurfelCloudType::Ptr scene_surfels(
          boost::make_shared<modelify::PointSurfelCloudType>(
              estimateNormals(scene_pointcloud_, verbose_)));
      // Validate initial alignment
      modelify::registration_toolbox::ICPParams icp_params;
      double cloud_resolution = modelify::kInvalidCloudResolution;
      double mean_squared_distance;
      double inlier_ratio;
      std::vector<size_t> outlier_indices;
      bool success = false;
      try {
        modelify::registration_toolbox::validateAlignment<modelify::PointSurfelType>(
            object_surfels_, detection_surfels, T_object_detection_init.getTransformationMatrix(),
            icp_params, cloud_resolution, &mean_squared_distance, &inlier_ratio, &outlier_indices);
        if (verbose_) {
          LOG(INFO) << "Initial validation results: \n"
                    << mean_squared_distance << " mean squared distance, " << inlier_ratio
                    << " inlier ratio";
        }
        success = true;
      } catch (...) {
        LOG(ERROR) << "Validation of alignment failed!";
      }

      // Refine transformation with ICP
      Transformation T_icp;
      if (use_icp_on_pointcloud_) {
        T_icp = icp(object_pointcloud_, scene_pointcloud_, T_object_detection_init, verbose_);
      } else {
        T_icp = icp(mesh_model_, scene_pointcloud_, T_object_detection_init, icp_config_file_,
                    verbose_);
      }

      // Validate alignment ICP
      double mean_squared_distance_icp;
      double inlier_ratio_icp;
      try {
        CHECK(detection_surfels);
        modelify::registration_toolbox::validateAlignment<modelify::PointSurfelType>(
            object_surfels_, detection_surfels, T_icp.getTransformationMatrix(), icp_params,
            cloud_resolution, &mean_squared_distance_icp, &inlier_ratio_icp, &outlier_indices);
        if (verbose_) {
          LOG(INFO) << "ICP validation results: \n"
                    << mean_squared_distance_icp << " mean squared distance, " << inlier_ratio_icp
                    << " inlier ratio";
        }
        success &= true;
      } catch (...) {
        LOG(ERROR) << "Validation of alignment failed!";
      }

      // Set refined transformation
      if (inlier_ratio_icp >= inlier_ratio || !success) {
        // TODO(gasserl): include mean_squared_distance_icp < mean_squared_distance
        T_object_detection = T_icp;
      } else {
        LOG(WARNING) << "ICP didn't improve alignment!";
        T_object_detection = T_object_detection_init;
      }

      if (!T_object_detection.getTransformationMatrix().allFinite()) {
        LOG(ERROR) << "Transformation from ICP is not valid!\n" << T_object_detection;
        return true;
      }

      visualizePointcloud(object_pointcloud_, detection_stamp_, detection_frame_id_,
                          object_pointcloud_pub_);
    } else {
      // Get final alignment with ICP
      if (use_icp_on_pointcloud_) {
        T_object_detection =
            icp(object_pointcloud_, scene_pointcloud_, T_object_detection_init, verbose_);
      } else {
        T_object_detection = icp(mesh_model_, scene_pointcloud_, T_object_detection_init,
                                 icp_config_file_, verbose_);
      }
    }
  }

  // Update kalman filter
  if (use_kalman_filter_ && valid_transform_to_reference) {
    // Get measured state
    Transformation T_kalman(T_detection_world * T_object_detection);
    Eigen::VectorXf z_kalman(7);
    z_kalman << T_kalman.getPosition().x(), T_kalman.getPosition().y(), T_kalman.getPosition().z(),
        T_kalman.getRotation().w(), T_kalman.getRotation().x(), T_kalman.getRotation().y(),
        T_kalman.getRotation().z();

    // Perform Kalman update
    if (verbose_) {
      LOG(INFO) << "Kalman update:";
      LOG(INFO) << "Covariance P before:\n" << P_kalman_;
      LOG(INFO) << "Observation z:\n" << z_kalman.transpose();
      LOG(INFO) << "State x before:\n" << x_kalman_.transpose();
    }

    K_kalman_ = P_kalman_ * (P_kalman_ + R_kalman_).inverse();
    x_kalman_ = x_kalman_ + K_kalman_ * (z_kalman - x_kalman_);
    P_kalman_ = (Eigen::MatrixXf::Identity(7, 7) - K_kalman_) * P_kalman_;
    if (verbose_) {
      LOG(INFO) << "State x after:\n" << x_kalman_.transpose();
      LOG(INFO) << "Measurement uncertainty R:\n" << R_kalman_;
      LOG(INFO) << "Gain K:\n" << K_kalman_;
      LOG(INFO) << "Covariance P:\n" << P_kalman_;
    }

    T_kalman.getPosition().x() = x_kalman_(0);
    T_kalman.getPosition().y() = x_kalman_(1);
    T_kalman.getPosition().z() = x_kalman_(2);
    Eigen::VectorXf quaternion(4);
    quaternion << x_kalman_(3), x_kalman_(4), x_kalman_(5), x_kalman_(6);
    quaternion.normalize();
    T_kalman.getRotation().setValues(quaternion(0), quaternion(1), quaternion(2), quaternion(3));
  }

  // Inlier ratio filter
  if (use_inlier_ratio_filter_) {
    double inlier_ratio_icp =
        computeInlierRatio(use_icp_on_pointcloud_, object_surfels_, detection_surfels, *mesh_model_,
                           detection_pointcloud_, T_object_detection);
    if (verbose_) {
      LOG(INFO) << "Inlier ratio ICP:    " << inlier_ratio_icp;
      LOG(INFO) << "Inlier ratio filter: " << inlier_ratio_filter_;
    }

    // Check minimum inier ratio
    const double inlier_ratio = std::max(inlier_ratio_filter_, inlier_ratio_icp);
    if (inlier_ratio < min_inlier_ratio_) {
      LOG(ERROR) << "Inlier ratio too low! " << inlier_ratio << " < " << min_inlier_ratio_;
      return false;
    }

    // Update filter transform
    if (inlier_ratio_filter_ > inlier_ratio_icp) {
      T_object_detection = T_object_detection_filter_;
    } else {
      T_object_detection_filter_ = T_object_detection;
    }
  }

  if (verbose_) {
    LOG(INFO) << "Time matching total: "
              << std::chrono::duration<float>(std::chrono::steady_clock::now() - time_start).count()
              << " s";
  }

  // Publish transformations to TF
  Transformation T_object_world;
  if (publish_static_transform_ && valid_transform_to_reference) {
    T_object_world = T_detection_world * T_object_detection;
    publishTransformation(T_object_world, detection_stamp_, reference_frame_id_, object_frame_id_);
  } else {
    if (publish_static_transform_) {
      LOG(WARNING) << "Publishing relative transform from detection frame " << detection_frame_id_
                   << " to object frame " << object_frame_id_ << " instead.";
    }
    publishTransformation(T_object_detection, detection_stamp_, detection_frame_id_,
                          object_frame_id_);
  }

  // Visualize object
  visualizeMesh(mesh_model_, detection_stamp_, object_frame_id_, object_mesh_pub_);

  return true;
}

bool ObjectDetector3D::lookupTransform(const std::string& target_frame,
                                       const std::string& source_frame, const ros::Time& timestamp,
                                       Transformation& transform, ros::Time& stamp_transform) {
  std::string tf_error_str;
  if (tf_listener_.canTransform(target_frame, source_frame, timestamp, &tf_error_str)) {
    tf::StampedTransform transform_stamped;
    tf_listener_.lookupTransform(target_frame, source_frame, timestamp, transform_stamped);
    tf::transformTFToKindr(transform_stamped, &transform);
    stamp_transform = timestamp;
    return true;
  } else if (tf_listener_.canTransform(target_frame, source_frame, ros::Time(0), &tf_error_str)) {
    tf::StampedTransform transform_stamped;
    tf_listener_.lookupTransform(target_frame, source_frame, ros::Time(0), transform_stamped);
    LOG(WARNING) << "Using latest transformation betweeen frames " << target_frame << " and "
                 << source_frame << ", " << (detection_stamp_ - transform_stamped.stamp_).toSec()
                 << " s back.";
    tf::transformTFToKindr(transform_stamped, &transform);
    stamp_transform = transform_stamped.stamp_;
    return true;
  } else {
    LOG(ERROR) << "Could not get transform from frame " << target_frame << " to frame "
               << source_frame << " at time " << timestamp << "!\nMessage: " << tf_error_str;
    return false;
  }
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

void ObjectDetector3D::publishStaticTransformation(const Transformation& transform,
                                                   const ros::Time& stamp,
                                                   const std::string& parent_frame_id,
                                                   const std::string& child_frame_id) {
  static tf2_ros::StaticTransformBroadcaster tf_broadcaster;
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
    const modelify::PointSurfelCloudType::Ptr& object_keypoints,
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
    const auto& point_object = object_keypoints->points[it->index_query];
    point_msg.x = point_object.x;
    point_msg.y = point_object.y;
    point_msg.z = point_object.z;
    marker.points.push_back(point_msg);
    const auto& point_detection = detection_keypoints->points[it->index_match];
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

}  // namespace cad_percept::object_detection