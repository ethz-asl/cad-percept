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

namespace cad_percept {
namespace object_detection {

class ObjectDetector3D {
  typedef kindr::minimal::QuatTransformationTemplate<float> Transformation;
  typedef kindr::minimal::RotationQuaternionTemplate<float> Quaternion;
  typedef PointMatcher<float> PM;

 public:
  ObjectDetector3D(const ros::NodeHandle& nh,
                   const ros::NodeHandle& nh_private);
  ~ObjectDetector3D() = default;

  void objectDetectionCallback(const sensor_msgs::PointCloud2& cloud_msg_in);

 private:
  void getParamsFromRos();
  void subscribeToTopics();
  void advertiseTopics();

  void processObject();

  void processDetectionUsingPcaAndIcp();
  bool findInitialGuessUsingPca(Transformation* T_object_detection_init);
  bool performICP(const Transformation& T_object_detection_init,
                  Transformation* T_object_detection);
  void visualizeObjectMesh(const std::string& frame_id,
                           const ros::Publisher& publisher) const;
  void visualizeObjectPointcloud(const ros::Time& timestamp,
                                 const std::string& frame_id);

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

  cgal::MeshModel::Ptr mesh_model_;
  pcl::PointCloud<pcl::PointXYZ> object_pointcloud_;
  sensor_msgs::PointCloud2 object_pointcloud_msg_;

  sensor_msgs::PointCloud2 detection_pointcloud_msg_;
  std::string detection_frame_id_;
  pcl::PointCloud<pcl::PointXYZ> detection_pointcloud_;

  // Parameters
  std::string pointcloud_topic_;
  std::string object_frame_id_;
  int num_points_icp_;
};

}
}

#endif  // CAD_PERCEPT_CPT_OBJECT_DETECTION_INCLUDE_CPT_OBJECT_DETECTION_DETECTION_MATCHER_H_
