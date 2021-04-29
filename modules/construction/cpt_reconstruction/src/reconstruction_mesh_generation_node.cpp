#include <cpt_reconstruction/reconstruction_mesh_generation.h>

//#include <sstream>
//#include <string>
//#include "ros/ros.h"
//#include "std_msgs/String.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "reconstruction_mesh_generation_node");
  ros::NodeHandle nodeHandle;

  // std::string path_scan = "/home/philipp/Schreibtisch/scan23.ply";

  cad_percept::cpt_reconstruction::MeshGeneration meshGeneration(nodeHandle);

  return 0;
}
