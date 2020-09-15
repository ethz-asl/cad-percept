#ifndef CAD_PERCEPT_CPT_OBJECT_DETECTION_INCLUDE_CPT_OBJECT_DETECTION_DETECTION_MATCHER_H_
#define CAD_PERCEPT_CPT_OBJECT_DETECTION_INCLUDE_CPT_OBJECT_DETECTION_DETECTION_MATCHER_H_

#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_definitions/mesh_model.h>
#include <kindr/minimal/quat-transformation.h>
#include <modelify/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pointmatcher/PointMatcher.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/ColorRGBA.h>

namespace cad_percept {
namespace object_detection {

class ObjectDetector3D {
  typedef kindr::minimal::QuatTransformationTemplate<float> Transformation;
  typedef kindr::minimal::RotationQuaternionTemplate<float> Quaternion;
  typedef PointMatcher<float> PM;

  enum KeypointType { kIss = 0, kHarris, kUniform, kNumKeypointTypes };
  std::map<size_t, std::string> KeypointNames = {
      {static_cast<size_t>(KeypointType::kIss), "ISS"},
      {static_cast<size_t>(KeypointType::kHarris), "Harris"},
      {static_cast<size_t>(KeypointType::kUniform), "uniform"}};
  enum DescriptorType { kFpfh = 0, kShot, kNumDescriptorTypes };
  std::map<size_t, std::string> DescriptorNames = {
      {static_cast<size_t>(DescriptorType::kFpfh), "FPFH"},
      {static_cast<size_t>(DescriptorType::kShot), "SHOT"}};
  enum MatchingMethod { kConventional = 0, kFastGlobalRegistration, kNumMatchingMethods };
  std::map<size_t, std::string> MatchingMethodNames = {
      {static_cast<size_t>(MatchingMethod::kConventional), "conventional"},
      {static_cast<size_t>(MatchingMethod::kFastGlobalRegistration), "FGR"}};

 public:
  ObjectDetector3D(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  ~ObjectDetector3D() = default;

  void objectDetectionCallback(const sensor_msgs::PointCloud2& cloud_msg_in);

 private:
  void getParamsFromRos();
  void subscribeToTopics();
  void advertiseTopics();
  void processMesh();

  void processDetectionUsingPcaAndIcp();
  static Transformation alignDetectionUsingPcaAndIcp(
      const pcl::PointCloud<pcl::PointXYZ>& object_pointcloud,
      const pcl::PointCloud<pcl::PointXYZ>& detection_pointcloud);
  static Transformation alignDetectionUsingPcaAndIcp(
      const pcl::PointCloud<pcl::PointXYZ>& object_pointcloud,
      const pcl::PointCloud<pcl::PointXYZ>& detection_pointcloud, const std::string& config_file);
  static Transformation alignDetectionUsingPcaAndIcp(
      const pcl::PointCloud<pcl::PointXYZ>& object_pointcloud,
      const pcl::PointCloud<pcl::PointXYZ>& detection_pointcloud, const std::string& config_file,
      Transformation* T_object_detection_init);
  static Transformation pca(const pcl::PointCloud<pcl::PointXYZ>& object_pointcloud,
                            const pcl::PointCloud<pcl::PointXYZ>& detection_pointcloud);
  static Transformation icp(const pcl::PointCloud<pcl::PointXYZ>& object_pointcloud,
                            const pcl::PointCloud<pcl::PointXYZ>& detection_pointcloud,
                            const Transformation& T_object_detection_init,
                            const std::string& config_file);

  void processDetectionUsing3dFeatures();
  template <typename descriptor_type>
  static Transformation computeTransformUsing3dFeatures(
      MatchingMethod matching_method, const modelify::PointSurfelCloudType::Ptr& detection_surfels,
      const modelify::PointSurfelCloudType::Ptr& detection_keypoints,
      const typename pcl::PointCloud<descriptor_type>::Ptr& detection_descriptors,
      const modelify::PointSurfelCloudType::Ptr& object_surfels,
      const modelify::PointSurfelCloudType::Ptr& object_keypoints,
      const typename pcl::PointCloud<descriptor_type>::Ptr& object_descriptors,
      double similarity_threshold, const modelify::CorrespondencesTypePtr& correspondences);
  template <typename descriptor_type>
  static Transformation computeTransformUsingFgr(
      const modelify::PointSurfelCloudType::Ptr& detection_surfels,
      const modelify::PointSurfelCloudType::Ptr& detection_keypoints,
      const typename pcl::PointCloud<descriptor_type>::Ptr& detection_descriptors,
      const modelify::PointSurfelCloudType::Ptr& object_surfels,
      const modelify::PointSurfelCloudType::Ptr& object_keypoints,
      const typename pcl::PointCloud<descriptor_type>::Ptr& object_descriptors,
      const modelify::CorrespondencesTypePtr& correspondences);
  template <typename descriptor_type>
  static Transformation computeTransformUsingModelify(
      const modelify::PointSurfelCloudType::Ptr& detection_surfels,
      const modelify::PointSurfelCloudType::Ptr& detection_keypoints,
      const typename pcl::PointCloud<descriptor_type>::Ptr& detection_descriptors,
      const modelify::PointSurfelCloudType::Ptr& object_surfels,
      const modelify::PointSurfelCloudType::Ptr& object_keypoints,
      const typename pcl::PointCloud<descriptor_type>::Ptr& object_descriptors,
      double correspondence_threshold, const modelify::CorrespondencesTypePtr& correspondences);
  static Transformation computeTransformFromCorrespondences(
      const modelify::PointSurfelCloudType::Ptr& detection_keypoints,
      const modelify::PointSurfelCloudType::Ptr& object_keypoints,
      const modelify::CorrespondencesTypePtr& correspondences);
  static Transformation icpUsingModelify(
      const modelify::PointSurfelCloudType::Ptr& detection_surfels,
      const modelify::PointSurfelCloudType::Ptr& object_surfels,
      const Transformation& transform_init);

  template <typename descriptor_type>
  bool get3dFeatures(const pcl::PointCloud<pcl::PointXYZ>& pointcloud_xyz,
                     const modelify::PointSurfelCloudType::Ptr& pointcloud_surfel_ptr,
                     const modelify::PointSurfelCloudType::Ptr& keypoints,
                     const typename pcl::PointCloud<descriptor_type>::Ptr& descriptors);
  bool getKeypoints(const KeypointType& keypoint_type,
                    const modelify::PointSurfelCloudType::Ptr& pointcloud_surfel_ptr,
                    const modelify::PointSurfelCloudType::Ptr& keypoints);
  static bool getIssKeypoints(const modelify::PointSurfelCloudType::Ptr& pointcloud_surfel_ptr,
                              const modelify::PointSurfelCloudType::Ptr& keypoints);
  static bool getHarrisKeypoints(const modelify::PointSurfelCloudType::Ptr& pointcloud_surfel_ptr,
                                 const modelify::PointSurfelCloudType::Ptr& keypoints);
  static bool getUniformKeypoints(const modelify::PointSurfelCloudType::Ptr& pointcloud_surfel_ptr,
                                  const modelify::PointSurfelCloudType::Ptr& keypoints);
  template <typename descriptor_type>
  static void getDescriptors(const modelify::PointSurfelCloudType::Ptr& pointcloud_surfel_ptr,
                             const modelify::PointSurfelCloudType::Ptr& keypoints,
                             const typename pcl::PointCloud<descriptor_type>::Ptr& descriptors);

  static void visualizeMesh(const cgal::MeshModel::Ptr& mesh_model, const ros::Time& timestamp,
                            const std::string& frame_id, const ros::Publisher& publisher);
  static void visualizePointcloud(const pcl::PointCloud<pcl::PointXYZ>& pointcloud,
                                  const ros::Time& timestamp, const std::string& frame_id,
                                  const ros::Publisher& publisher);
  static void visualizeKeypoints(const modelify::PointSurfelCloudType::Ptr& keypoints,
                                 const ros::Time& timestamp, const std::string& frame_id,
                                 const ros::Publisher& publisher);
  void visualizeCorrespondences(const modelify::PointSurfelCloudType::Ptr& detection_keypoints,
                                const modelify::CorrespondencesTypePtr& correspondences,
                                const std::string& frame_id, const ros::Publisher& publisher);
  static void visualizeNormals(const modelify::PointSurfelCloudType::Ptr& surfels,
                               const std::string& marker_namespace, const std::string& frame_id,
                               const ros::Publisher& publisher);

  static void publishTransformation(const Transformation& transform, const ros::Time& stamp,
                                    const std::string& parent_frame_id,
                                    const std::string& child_frame_id);

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

  // Object
  cgal::MeshModel::Ptr mesh_model_;
  std::string object_frame_id_;
  pcl::PointCloud<pcl::PointXYZ> object_pointcloud_;
  modelify::PointSurfelCloudType::Ptr object_surfels_;
  modelify::PointSurfelCloudType::Ptr object_keypoints_;
  modelify::DescriptorFPFHCloudType::Ptr object_descriptors_fpfh_;
  modelify::DescriptorSHOTCloudType::Ptr object_descriptors_shot_;

  // Detection
  std::string pointcloud_topic_;
  ros::Time detection_stamp_;
  std::string detection_frame_id_;
  pcl::PointCloud<pcl::PointXYZ> detection_pointcloud_;

  // Parameters
  KeypointType keypoint_type_;
  DescriptorType descriptor_type_;
  MatchingMethod matching_method_;

  std::string icp_config_file_;
  float correspondence_threshold_;
  bool downsampling_;
};

}  // namespace object_detection
}  // namespace cad_percept

#endif  // CAD_PERCEPT_CPT_OBJECT_DETECTION_INCLUDE_CPT_OBJECT_DETECTION_DETECTION_MATCHER_H_
