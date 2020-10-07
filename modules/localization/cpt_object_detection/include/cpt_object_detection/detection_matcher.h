#ifndef CAD_PERCEPT_CPT_OBJECT_DETECTION_INCLUDE_CPT_OBJECT_DETECTION_DETECTION_MATCHER_H_
#define CAD_PERCEPT_CPT_OBJECT_DETECTION_INCLUDE_CPT_OBJECT_DETECTION_DETECTION_MATCHER_H_

#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_definitions/mesh_model.h>
#include <kindr/minimal/quat-transformation.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pointmatcher/PointMatcher.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace cad_percept {
namespace object_detection {

class ObjectDetector3D {
  typedef kindr::minimal::QuatTransformationTemplate<float> Transformation;
  typedef kindr::minimal::RotationQuaternionTemplate<float> Quaternion;
  typedef PointMatcher<float> PM;

 public:
  ObjectDetector3D(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  ~ObjectDetector3D() = default;

  void objectDetectionCallback(const sensor_msgs::PointCloud2& cloud_msg_in);

 private:
  void getParamsFromRos();
  void subscribeToTopics();
  void advertiseTopics();

  void processDetectionUsingPcaAndIcp();
  static Transformation alignDetectionUsingPcaAndIcp(
      const cgal::MeshModel::Ptr& mesh_model,
      const pcl::PointCloud<pcl::PointXYZ>& detection_pointcloud);
  static Transformation alignDetectionUsingPcaAndIcp(
      const cgal::MeshModel::Ptr& mesh_model,
      const pcl::PointCloud<pcl::PointXYZ>& detection_pointcloud, const std::string& config_file);
  static Transformation alignDetectionUsingPcaAndIcp(
      const cgal::MeshModel::Ptr& mesh_model,
      const pcl::PointCloud<pcl::PointXYZ>& detection_pointcloud, const std::string& config_file,
      Transformation* T_object_detection_init);
  static Transformation pca(const cgal::MeshModel::Ptr& mesh_model,
                            const pcl::PointCloud<pcl::PointXYZ>& detection_pointcloud);
  static Transformation icp(const cgal::MeshModel::Ptr& mesh_model,
                            const pcl::PointCloud<pcl::PointXYZ>& detection_pointcloud,
                            const Transformation& T_object_detection_init,
                            const std::string& config_file);

  static PM::DataPoints sampleDataPointsFromMesh(const cgal::MeshModel::Ptr& mesh_model,
                                                 const int number_of_points);
  static PM::DataPoints convertMeshToDataPoints(const cgal::MeshModel::Ptr& mesh_model);
  static PM::DataPoints convertMeshPointsToDataPoints(const cgal::MeshModel::Ptr& mesh_model,
                                                      const std::vector<cgal::Point>& points);

  static void visualizeMesh(const cgal::MeshModel::Ptr& mesh_model, const ros::Time& timestamp,
                            const std::string& frame_id, const ros::Publisher& publisher);
  static void visualizePointcloud(const pcl::PointCloud<pcl::PointXYZ>& pointcloud,
                                  const ros::Time& timestamp, const std::string& frame_id,
                                  const ros::Publisher& publisher);
  static void publishTransformation(const Transformation& transform, const ros::Time& stamp,
                                    const std::string& parent_frame_id,
                                    const std::string& child_frame_id);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber detection_pointcloud_sub_;
  ros::Publisher object_mesh_pub_;
  ros::Publisher object_mesh_init_pub_;

  // Object
  cgal::MeshModel::Ptr mesh_model_;
  std::string object_frame_id_;

  // Detection
  std::string pointcloud_topic_;
  ros::Time detection_stamp_;
  std::string detection_frame_id_;
  pcl::PointCloud<pcl::PointXYZ> detection_pointcloud_;

  // Parameters
  std::string icp_config_file_;
};

}  // namespace object_detection
}  // namespace cad_percept

#endif  // CAD_PERCEPT_CPT_OBJECT_DETECTION_INCLUDE_CPT_OBJECT_DETECTION_DETECTION_MATCHER_H_
