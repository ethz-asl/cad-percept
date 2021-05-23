#include "cpt_reconstruction/reconstruction_mesh_generation.h"

namespace cad_percept {
namespace cpt_reconstruction {
MeshGeneration::MeshGeneration(ros::NodeHandle nodeHandle)
    : nodeHandle_(nodeHandle) {
  subscriber_ = nodeHandle_.subscribe("classified_shapes", 1000,
                                      &MeshGeneration::messageCallback, this);
}

void MeshGeneration::messageCallback(const ::cpt_reconstruction::shape &msg) {}

}  // namespace cpt_reconstruction
}  // namespace cad_percept