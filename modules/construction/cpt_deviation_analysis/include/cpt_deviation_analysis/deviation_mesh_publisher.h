#ifndef CPT_DEVIATION_ANALYSIS_INCLUDE_CPT_DEVIATION_ANALYSIS_MESH_PUBLISHER_H_
#define CPT_DEVIATION_ANALYSIS_INCLUDE_CPT_DEVIATION_ANALYSIS_MESH_PUBLISHER_H_

#include <CGAL/IO/Polyhedron_iostream.h>
#include <cgal_conversions/eigen_conversions.h>
#include <cgal_conversions/mesh_conversions.h>
#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_definitions/mesh_model.h>
#include <cgal_msgs/FacetID.h>
#include <cgal_msgs/PublishMesh.h>
#include <cgal_msgs/SetDeviationPlane.h>
#include <cgal_msgs/TriangleMeshStamped.h>
#include <cpt_selective_icp/References.h>
#include <cpt_utils/mesh_publisher.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

namespace cad_percept {
namespace cpt_deviation_analysis {

/*
 * Simple node that reads off files and publishes them as TriangleMeshStamped.
 */
class DeviationMeshPublisher : public cad_percept::cpt_utils::MeshPublisher {
 public:
  DeviationMeshPublisher(ros::NodeHandle nh, ros::NodeHandle nh_private);

  /*
   * Service callback for publishing a mesh file.
   * Currently it always calls publishOffFile with the default_filename_.
   */
  bool triggerPublishMesh(cgal_msgs::PublishMesh::Request &req,
                          cgal_msgs::PublishMesh::Response &res);
  void jsonListener(const std_msgs::String &msg);

 private:
  /*
   * Reads json and publishes it via pub_mesh_
   */
  bool publishMesh(nlohmann::json &j);

  ros::Subscriber json_listener_;
  ros::ServiceClient set_references_;
  ros::ServiceClient set_deviation_target_;
};
}  // namespace cpt_deviation_analysis
}  // namespace cad_percept
#endif  // CPT_DEVIATION_ANALYSIS_INCLUDE_CPT_DEVIATION_ANALYSIS_MESH_PUBLISHER_H_
