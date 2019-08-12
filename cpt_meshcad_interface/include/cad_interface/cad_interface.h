#ifndef CAD_INTERFACE_CAD_INTERFACE_H_
#define CAD_INTERFACE_CAD_INTERFACE_H_

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_msgs/TriangleMeshStamped.h>
#include <cgal_msgs/TriangleMesh.h>
#include <cgal_conversions/mesh_conversions.h>

namespace cad_percept {
namespace cad_interface {

class CadInterface {
 public:
  CadInterface(ros::NodeHandle &n);
  ~CadInterface();

  void load(const std::string &filename);

  void publishPointCloudThread();
 private:
  ros::Publisher cad_mesh_pub_;
  cad_percept::cgal::Polyhedron P_;
  ros::NodeHandle &nh_;
  int iterator_;
  cgal_msgs::TriangleMeshStamped cad_mesh_msg_;
};

} //namespace cad_interface
}

#endif /* CAD_INTERFACE_CAD_INTERFACE_H_ */
