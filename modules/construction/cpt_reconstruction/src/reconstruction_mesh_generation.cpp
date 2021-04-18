#include <cpt_reconstruction/reconstruction_mesh_generation.h>

#include <geometry_msgs/Vector3.h>
#include "cpt_reconstruction/shape.h"
#include "std_msgs/String.h"

namespace cad_percept {
namespace cpt_reconstruction {
MeshGeneration::MeshGeneration(ros::NodeHandle nodeHandle)
    : nodeHandle_(nodeHandle) {
  subscriber_ = nodeHandle_.subscribe("ransac_shape", 10,
                                      &MeshGeneration::messageCallback, this);
  ros::spin();
}

void MeshGeneration::messageCallback(const ::cpt_reconstruction::shape& msg) {
  ROS_INFO("[Mesh Generation] Id: %d with size: %d", msg.id,
           msg.vectors.size());

  // Plane
  if (msg.id == 0){
    std::vector<geometry_msgs::Vector3> pub_vectors = msg.vectors;
    for (unsigned i = 0; i < pub_vectors.size(); i++) {
      geometry_msgs::Vector3 v = pub_vectors.at(i);
      // Meshing here
    }
  } else if (msg.id == 0){

  } else {
    ROS_INFO("Unknown shape\n");
  }
}
}  // namespace cpt_reconstruction
}  // namespace cad_percept