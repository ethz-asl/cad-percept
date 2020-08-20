#ifndef CAD_PERCEPT_CPT_OBJECT_DETECTION_INCLUDE_CPT_OBJECT_DETECTION_DETECTION_MATCHER_H_
#define CAD_PERCEPT_CPT_OBJECT_DETECTION_INCLUDE_CPT_OBJECT_DETECTION_DETECTION_MATCHER_H_

#include <ros/ros.h>
#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_definitions/mesh_model.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

namespace cad_percept {
namespace object_detection {

class DetectionMatcher {
 public:
  DetectionMatcher(const ros::NodeHandle& nh,
                   const ros::NodeHandle& nh_private);
  ~DetectionMatcher() = default;

  void pointcloudCallback(const sensor_msgs::PointCloud2& cloud_msg_in);

 private:
  void subscribeToTopics();
  void advertiseTopics();

  void visualizeObjectMesh(const std::string& frame_id,
                           const ros::Publisher& publisher) const;
  void visualizeObjectPointcloud();
  
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber detection_pointcloud_sub_;
  ros::Publisher object_pointcloud_pub_;
  ros::Publisher object_mesh_pub_;

  sensor_msgs::PointCloud2 detection_pointcloud_msg_;
  sensor_msgs::PointCloud2 object_pointcloud_msg_;
  pcl::PointCloud<pcl::PointXYZ> detection_pointcloud_;
  pcl::PointCloud<pcl::PointXYZ> object_pointcloud_;

  std::string detection_frame_id_;
  cgal::MeshModel mesh_model_;
};

}
}

#endif  // CAD_PERCEPT_CPT_OBJECT_DETECTION_INCLUDE_CPT_OBJECT_DETECTION_DETECTION_MATCHER_H_
