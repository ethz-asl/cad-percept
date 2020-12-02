#ifndef CPT_OBJECT_DETECTION_OBJECT_DETECTOR_3D_H_
#define CPT_OBJECT_DETECTION_OBJECT_DETECTOR_3D_H_

#include <cpt_object_detection/learned_descriptor.h>
#include <cpt_object_detection/object_detection.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>

namespace cad_percept::object_detection {

class ObjectDetector3D {
  std::map<size_t, std::string> KeypointNames = {
      {static_cast<size_t>(KeypointType::kIss), "ISS"},
      {static_cast<size_t>(KeypointType::kHarris), "Harris"},
      {static_cast<size_t>(KeypointType::kUniform), "uniform"}};
  std::map<size_t, std::string> DescriptorNames = {
      {static_cast<size_t>(DescriptorType::kFpfh), "FPFH"},
      {static_cast<size_t>(DescriptorType::kShot), "SHOT"},
      {static_cast<size_t>(DescriptorType::k3dSmoothNet), "3DSmoothNet"}};
  std::map<size_t, std::string> MatchingMethodNames = {
      {static_cast<size_t>(MatchingMethod::kGeometricConsistency), "geometric_consistency"},
      {static_cast<size_t>(MatchingMethod::kFastGlobalRegistration), "FGR"},
      {static_cast<size_t>(MatchingMethod::kTeaser), "Teaser"}};

 public:
  ObjectDetector3D(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  ~ObjectDetector3D() = default;

  void objectDetectionCallback(const sensor_msgs::PointCloud2& cloud_msg_in);
  bool startInitializationCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  bool initializeObjectMesh();
  bool processDetectionUsingInitializationAndIcp(Transformation* T_object_world);
  Transformation processDetectionUsingPcaAndIcp();
  Transformation processDetectionUsing3dFeatures();
  void processInitialization();

  bool lookupTransform(const std::string& target_frame, const std::string& source_frame,
                       const ros::Time& timestamp, Transformation& transform,
                       ros::Time& stamp_transform);

  static void visualizeMesh(const cgal::MeshModel::Ptr& mesh_model, const ros::Time& timestamp,
                            const std::string& frame_id, const ros::Publisher& publisher);
  static void visualizePointcloud(const pcl::PointCloud<pcl::PointXYZ>& pointcloud,
                                  const ros::Time& timestamp, const std::string& frame_id,
                                  const ros::Publisher& publisher);
  static void visualizeKeypoints(const modelify::PointSurfelCloudType::Ptr& keypoints,
                                 const ros::Time& timestamp, const std::string& frame_id,
                                 const ros::Publisher& publisher);
  static void visualizeCorrespondences(
      const modelify::PointSurfelCloudType::Ptr& detection_keypoints,
      const modelify::PointSurfelCloudType::Ptr& object_keypoints,
      const modelify::CorrespondencesTypePtr& correspondences, const std::string& frame_id,
      const ros::Publisher& publisher);
  static void visualizeNormals(const modelify::PointSurfelCloudType::Ptr& surfels,
                               const std::string& marker_namespace, const std::string& frame_id,
                               const ros::Publisher& publisher);
  static void publishTransformation(const Transformation& transform, const ros::Time& stamp,
                                    const std::string& parent_frame_id,
                                    const std::string& child_frame_id);
  static void publishStaticTransformation(const Transformation& transform, const ros::Time& stamp,
                                          const std::string& parent_frame_id,
                                          const std::string& child_frame_id);

 private:
  void getParamsFromRos();
  void subscribeToTopics();
  void advertiseTopics();

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber detection_pointcloud_sub_;
  ros::Publisher object_pointcloud_pub_;
  ros::Publisher object_mesh_pub_;
  ros::Publisher object_mesh_init_pub_;
  ros::Publisher object_keypoint_pub_;
  ros::Publisher detection_keypoint_pub_;
  ros::Publisher correspondences_pub_;
  ros::Publisher normals_pub_;
  ros::Publisher initialization_detection_pointcloud_pub_;
  ros::ServiceServer initialization_srv_;

  tf::TransformListener tf_listener_;

  // Object
  cgal::MeshModel::Ptr mesh_model_;
  std::string object_frame_id_;
  pcl::PointCloud<pcl::PointXYZ> object_pointcloud_;
  modelify::PointSurfelCloudType::Ptr object_surfels_;
  modelify::PointSurfelCloudType::Ptr object_keypoints_;
  modelify::DescriptorFPFHCloudType::Ptr object_descriptors_fpfh_;
  modelify::DescriptorSHOTCloudType::Ptr object_descriptors_shot_;
  pcl::PointCloud<LearnedDescriptor>::Ptr object_descriptors_learned_;

  // Detection
  std::string pointcloud_topic_;
  ros::Time detection_stamp_;
  std::string detection_frame_id_;
  pcl::PointCloud<pcl::PointXYZ> detection_pointcloud_;

  // Initialization
  bool initialized_;
  bool initializing_;
  ros::Time initialization_start_time_;
  float initialization_duration_s_;
  std::vector<pcl::PointCloud<pcl::PointXYZ>> initialization_object_pointclouds_;
  std::vector<Transformation> initialization_object_poses_;
  std::vector<Transformation> initialization_detection_poses_;
  Transformation initialization_object_pose_;

  // Parameters: General options
  bool use_3d_features_;
  bool publish_static_transform_;
  std::string reference_frame_id_;

  // Parameters: 3D Features
  KeypointType keypoint_type_;
  DescriptorType descriptor_type_;
  MatchingMethod matching_method_;
  float correspondence_threshold_;
  float downsampling_resolution_;

  // Parameters: ICP
  bool refine_using_icp_;
  bool use_icp_on_pointcloud_;
  std::string icp_config_file_;
};

}  // namespace cad_percept::object_detection

#endif  // CPT_OBJECT_DETECTION_OBJECT_DETECTOR_3D_H_
