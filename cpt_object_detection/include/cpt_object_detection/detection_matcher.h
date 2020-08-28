#ifndef CAD_PERCEPT_CPT_OBJECT_DETECTION_INCLUDE_CPT_OBJECT_DETECTION_DETECTION_MATCHER_H_
#define CAD_PERCEPT_CPT_OBJECT_DETECTION_INCLUDE_CPT_OBJECT_DETECTION_DETECTION_MATCHER_H_

#include <ros/ros.h>
#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_definitions/mesh_model.h>
#include <kindr/minimal/quat-transformation.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pointmatcher/PointMatcher.h>
#include <sensor_msgs/PointCloud2.h>

#include <modelify/common.h>

namespace cad_percept {
namespace object_detection {

class DetectionMatcher {
  typedef kindr::minimal::QuatTransformationTemplate<float> Transformation;
  typedef kindr::minimal::RotationQuaternionTemplate<float> Quaternion;
  typedef PointMatcher<float> PM;
 private:
  enum KeypointType {
    kIss = 0,
    kHarris,
    kUniform,
    kNumKeypointTypes
  };
  std::map<size_t, std::string> KeypointNames = {
      {static_cast<size_t>(KeypointType::kIss), "ISS"},
      {static_cast<size_t>(KeypointType::kHarris), "Harris"},
      {static_cast<size_t>(KeypointType::kUniform), "Uniform"}
  };
  enum DescriptorType {
    kFpfh = 0,
    kShot,
    kNumDescriptorTypes
  };
  std::map<size_t, std::string> DescriptorNames = {
      {static_cast<size_t>(DescriptorType::kFpfh), "FPFH"},
      {static_cast<size_t>(DescriptorType::kShot), "SHOT"}
  };

 public:
  DetectionMatcher(const ros::NodeHandle& nh,
                   const ros::NodeHandle& nh_private);
  ~DetectionMatcher() = default;

  void pointcloudCallback(const sensor_msgs::PointCloud2& cloud_msg_in);

 private:
  void getParamsFromRos();
  void subscribeToTopics();
  void advertiseTopics();

  void processObject();

  void processPointcloudUsingPcaAndIcp();
  bool findInitialGuessUsingPca(Transformation* T_object_detection_init);
  bool performICP(const Transformation& T_object_detection_init,
                  Transformation* T_object_detection);

  void processPointcloudUsing3dFeatures();

  template <typename descriptor_type>
  bool computeTransformUsing3dFeatures(
      const typename pcl::PointCloud<descriptor_type>::Ptr& object_descriptors,
      Transformation* T_object_detection);

  template <typename descriptor_type>
  bool get3dFeatures(
      const pcl::PointCloud<pcl::PointXYZ>& pointcloud_xyz,
      const modelify::PointSurfelCloudType::Ptr& pointcloud_surfel_ptr,
      const modelify::PointSurfelCloudType::Ptr& keypoints,
      const typename pcl::PointCloud<descriptor_type>::Ptr& descriptors);

  bool getKeypoints(
      const KeypointType& keypoint_type,
      const modelify::PointSurfelCloudType::Ptr& pointcloud_surfel_ptr,
      const modelify::PointSurfelCloudType::Ptr& keypoints);
  static bool getIssKeypoints(
      const modelify::PointSurfelCloudType::Ptr& pointcloud_surfel_ptr,
      const modelify::PointSurfelCloudType::Ptr& keypoints);
  static bool getHarrisKeypoints(
      const modelify::PointSurfelCloudType::Ptr& pointcloud_surfel_ptr,
      const modelify::PointSurfelCloudType::Ptr& keypoints);
  static bool getUniformKeypoints(
      const modelify::PointSurfelCloudType::Ptr& pointcloud_surfel_ptr,
      const modelify::PointSurfelCloudType::Ptr& keypoints);

  template <typename descriptor_type>
  static void getDescriptors(
      const modelify::PointSurfelCloudType::Ptr& pointcloud_surfel_ptr,
      const modelify::PointSurfelCloudType::Ptr& keypoints,
      const typename pcl::PointCloud<descriptor_type>::Ptr& descriptors);

  bool computeTransformFromCorrespondences(
      const modelify::PointSurfelCloudType::Ptr& detection_surfels,
      const modelify::PointSurfelCloudType::Ptr& detection_keypoints,
      const modelify::CorrespondencesTypePtr& correspondences,
      Transformation* transform);

  void visualizeObjectMesh(const ros::Time& timestamp,
                           const std::string& frame_id,
                           const ros::Publisher& publisher) const;
  void visualizeObjectPointcloud(const ros::Time& timestamp,
                                 const std::string& frame_id);
  void visualizeCorrespondences(
      const modelify::PointSurfelCloudType::Ptr& detection_keypoints,
      const modelify::CorrespondencesTypePtr& correspondences,
      const ros::Time& timestamp, const std::string& frame_id,
      const ros::Publisher& publisher);
  static void visualizeNormals(
      const modelify::PointSurfelCloudType::Ptr& surfels,
      const std::string& marker_namespace, const ros::Time& timestamp,
      const std::string& frame_id, const ros::Publisher& publisher);

  static void publishTransformation(
      const Transformation& transform,
      const ros::Time& stamp, const std::string& parent_frame_id,
      const std::string& child_frame_id) ;
  
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

  cgal::MeshModel mesh_model_;
  pcl::PointCloud<pcl::PointXYZ> object_pointcloud_;
  sensor_msgs::PointCloud2 object_pointcloud_msg_;
  modelify::PointSurfelCloudType::Ptr object_surfels_;
  modelify::PointSurfelCloudType::Ptr object_keypoints_;
  modelify::DescriptorFPFHCloudType::Ptr object_descriptors_fpfh_;
  modelify::DescriptorSHOTCloudType::Ptr object_descriptors_shot_;

  sensor_msgs::PointCloud2 detection_pointcloud_msg_;
  std::string detection_frame_id_;
  pcl::PointCloud<pcl::PointXYZ> detection_pointcloud_;

  // Parameters
  std::string object_frame_id_;
  int num_points_icp_;
  KeypointType keypoint_type_;
  DescriptorType descriptor_type_;
};

}
}

#endif  // CAD_PERCEPT_CPT_OBJECT_DETECTION_INCLUDE_CPT_OBJECT_DETECTION_DETECTION_MATCHER_H_
