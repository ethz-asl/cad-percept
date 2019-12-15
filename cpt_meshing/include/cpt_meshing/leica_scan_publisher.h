#ifndef CPT_UTILS_INCLUDE_CPT_UTILS_LEICA_SCAN_PUBLISHER_H_
#define CPT_UTILS_INCLUDE_CPT_UTILS_LEICA_SCAN_PUBLISHER_H_

#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_definitions/mesh_model.h>
#include <cgal_msgs/FacetID.h>
#include <cgal_msgs/PublishMesh.h>
#include <cgal_msgs/TriangleMeshStamped.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

namespace cad_percept {
namespace meshing {

/*
 * Simple node that reads off files and publishes them as TriangleMeshStamped.
 */
class LeicaScanPublisher {
 public:
  LeicaScanPublisher(ros::NodeHandle nh, ros::NodeHandle nh_private);

  bool createMeshModel();

 protected:
  struct LeicaFileLine {
    float x;
    float y;
    float z;
    int snr;
    int r;
    int g;
    int b;
  };
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::ServiceServer publish_service_;
  ros::Publisher pub_mesh_;
  ros::Publisher pub_pcl_;
  std::string default_filename_;
  std::string frame_name_;

  cgal::MeshModel::Ptr mesh_model_;
};
}  // namespace meshing
}  // namespace cad_percept
#endif  // CPT_UTILS_INCLUDE_CPT_UTILS_LEICA_SCAN_PUBLISHER_H_
