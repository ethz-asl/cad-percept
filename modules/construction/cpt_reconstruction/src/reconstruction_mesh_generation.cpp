#include <cpt_reconstruction/reconstruction_mesh_generation.h>

#include "std_msgs/String.h"

namespace cad_percept {
namespace cpt_reconstruction {
MeshGeneration::MeshGeneration(ros::NodeHandle nodeHandle)
    : nodeHandle_(nodeHandle) {
  subscriber_ = nodeHandle_.subscribe("ransac_shape", 10,
                                      &MeshGeneration::messageCallback, this);
  ros::spin();
}

void MeshGeneration::messageCallback(const std_msgs::String& msg) {
  ROS_INFO("[Mesh Generation]\n");
  ROS_INFO("[Mesh Generation] %s", msg.data.c_str());
}
}  // namespace cpt_reconstruction
}  // namespace cad_percept