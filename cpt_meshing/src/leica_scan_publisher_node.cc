#include <cpt_meshing/leica_scan_publisher.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "mesh_publisher_node");
  // Creating the node handles
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Create Node object
  cad_percept::meshing::LeicaScanPublisher node(nh, nh_private);

  ros::spin();

  return 0;
}