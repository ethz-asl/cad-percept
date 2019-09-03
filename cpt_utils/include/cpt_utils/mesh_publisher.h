#ifndef CPT_UTILS_INCLUDE_CPT_UTILS_MESH_PUBLISHER_H_
#define CPT_UTILS_INCLUDE_CPT_UTILS_MESH_PUBLISHER_H_

#include <cgal_conversions/eigen_conversions.h>
#include <cgal_conversions/mesh_conversions.h>
#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_definitions/mesh_model.h>
#include <cgal_msgs/PublishMesh.h>
#include <cgal_msgs/TriangleMeshStamped.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

namespace cad_percept {
namespace cpt_utils {

/*
 * Simple node that reads off files and publishes them as TriangleMeshStamped.
 */
class MeshPublisher {
 public:
  MeshPublisher(ros::NodeHandle nh, ros::NodeHandle nh_private);

  /*
   * Reads off file and publishes it via pub_mesh_
   */
  bool publishOffFile(std::string filename = std::string());

  /*
   * Service callback.
   * Currently it always calls publishOffFile with the default_filename_.
   */
  bool triggerService(cgal_msgs::PublishMesh::Request &req, cgal_msgs::PublishMesh::Response &res);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::ServiceServer publish_service_;
  ros::Publisher pub_mesh_;
  std::string default_filename_;
  std::string frame_name_;
};
}  // namespace cpt_utils
}  // namespace cad_percept
#endif  // CPT_UTILS_INCLUDE_CPT_UTILS_MESH_PUBLISHER_H_
