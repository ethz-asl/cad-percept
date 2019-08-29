#ifndef CPT_UTILS_INCLUDE_CPT_UTILS_MESH_PUBLISHER_H_
#define CPT_UTILS_INCLUDE_CPT_UTILS_MESH_PUBLISHER_H_

#include <cgal_conversions/eigen_conversions.h>
#include <cgal_conversions/mesh_conversions.h>
#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_definitions/mesh_model.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

namespace cad_percept {
namespace cpt_utils {

class MeshPublisher {
 public:
  MeshPublisher(ros::NodeHandle nh, ros::NodeHandle nh_private) : nh_(nh), nh_private_(nh_private) {
    pub_mesh_ = nh_.advertise<cgal_msgs::TriangleMesh>("mesh_out", 1, true);
    publish_service_ = nh_.advertiseService("publish", &MeshPublisher::triggerService, this);
    default_filename_ = nh_private_.param<std::string>("default_filename", "mesh.off");
  }

  bool publishOffFile(const std::string filename);
  bool triggerService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::ServiceServer publish_service_;
  ros::Publisher pub_mesh_;
  std::string default_filename_;
};
}  // namespace cpt_utils
}  // namespace cad_percept
#endif  // CPT_UTILS_INCLUDE_CPT_UTILS_MESH_PUBLISHER_H_
