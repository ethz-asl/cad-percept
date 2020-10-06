#include "cpt_object_detection/object_detector_3d.h"

#include <cgal_conversions/mesh_conversions.h>
#include <cgal_msgs/TriangleMeshStamped.h>
#include <cpt_utils/pc_processing.h>
#include <pcl/filters/voxel_grid.h>
#include <minkindr_conversions/kindr_msg.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

namespace cad_percept {
namespace object_detection {

ObjectDetector3D::ObjectDetector3D(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      object_frame_id_("object_detection_mesh"),
      pointcloud_topic_("/camera/depth/color/points"),
      detection_frame_id_("camera_depth_optical_frame"),
      keypoint_type_(kIss),
      descriptor_type_(kFpfh),
      matching_method_(kConventional),
      use_3d_features_(true),
      refine_using_icp_(true),
      correspondence_threshold_(0.1),
      downsampling_resolution_(0.001) {
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
  nh_private_.param("use_3d_features", use_3d_features_, use_3d_features_);
  nh_private_.param("refine_using_icp", refine_using_icp_, refine_using_icp_);
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
  if (use_3d_features_) {
    object_surfels_.reset(new modelify::PointSurfelCloudType());
    object_keypoints_.reset(new modelify::PointSurfelCloudType());
    switch (descriptor_type_) {
      case kFpfh:
        object_descriptors_fpfh_.reset(new modelify::DescriptorFPFHCloudType());
        get3dFeatures<modelify::DescriptorFPFH>(keypoint_type_, object_pointcloud_, object_surfels_,
                                                object_keypoints_, object_descriptors_fpfh_);
        break;
      case kShot:
        object_descriptors_shot_.reset(new modelify::DescriptorSHOTCloudType());
        get3dFeatures<modelify::DescriptorSHOT>(keypoint_type_, object_pointcloud_, object_surfels_,
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
  }

  // Visualize object
  bool visualize_object_on_startup = false;
  nh_private_.param("visualize_object_on_startup", visualize_object_on_startup,
                    visualize_object_on_startup);
  if (visualize_object_on_startup) {
    visualizePointcloud(object_pointcloud_, ros::Time::now(), detection_frame_id_,
                        object_pointcloud_pub_);
    visualizeMesh(mesh_model_, ros::Time::now(), detection_frame_id_, object_mesh_init_pub_);
    if (use_3d_features_) {
      visualizeKeypoints(object_keypoints_, ros::Time::now(), detection_frame_id_,
                         object_keypoint_pub_);
    }
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

// TODO(gasserl): another child class?
void ObjectDetector3D::processDetectionUsing3dFeatures() {
  std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

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
    LOG(INFO) << "Detection pointcloud downsampled to resolution of " << downsampling_resolution_
              << " m, resulting in " << detection_pointcloud_.size() << " points";
    LOG(INFO)
        << "Time downsampling: "
        << std::chrono::duration<float>(std::chrono::steady_clock::now() - start_sampling).count();
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
      get3dFeatures<modelify::DescriptorFPFH>(keypoint_type_, detection_pointcloud_,
                                              detection_surfels, detection_keypoints,
                                              detection_descriptors_fpfh);
      T_features = computeTransformUsing3dFeatures<modelify::DescriptorFPFH>(
          matching_method_, detection_surfels, detection_keypoints, detection_descriptors_fpfh,
          object_surfels_, object_keypoints_, object_descriptors_fpfh_, correspondence_threshold_,
          correspondences);
      break;
    case kShot:
      get3dFeatures<modelify::DescriptorSHOT>(keypoint_type_, detection_pointcloud_,
                                              detection_surfels, detection_keypoints,
                                              detection_descriptors_shot);
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

  // Publish initial transform
  publishTransformation(T_features.inverse(), detection_stamp_, detection_frame_id_,
                        object_frame_id_ + "_init");
  visualizeMesh(mesh_model_, detection_stamp_, object_frame_id_ + "_init", object_mesh_init_pub_);

  // Refine using ICP
  Transformation T_icp = T_features;
  if (refine_using_icp_) {
    T_icp = icpUsingModelify(detection_surfels, object_surfels_, T_features);
  }

  if (!T_icp.getTransformationMatrix().allFinite()) {
    LOG(ERROR) << "Transformation from ICP is not valid!\n" << T_icp;
    return;
  }

  // Publish results
  publishTransformation(T_icp.inverse(), detection_stamp_, detection_frame_id_, object_frame_id_);
  visualizePointcloud(object_pointcloud_, detection_stamp_, detection_frame_id_,
                      object_pointcloud_pub_);
  visualizeMesh(mesh_model_, detection_stamp_, object_frame_id_, object_mesh_pub_);
  LOG(INFO) << "Published results.";
  LOG(INFO) << "Time aligning: "
            << std::chrono::duration<float>(std::chrono::steady_clock::now() - start).count()
            << " s";
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
    const auto& point_object = object_keypoints->points[it->index_match];
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