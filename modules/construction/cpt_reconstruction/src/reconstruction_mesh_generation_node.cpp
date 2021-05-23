#include <cpt_reconstruction/reconstruction_mesh_generation.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "reconstruction_mesh_generation_node");
  ros::NodeHandle nodeHandle;

  cad_percept::cpt_reconstruction::MeshGeneration mesh_generation();

  return 0;
}