#include <cpt_deviation_analysis/deviation_mesh_publisher.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "mesh_publisher_node");
  // Creating the node handles
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Create Node object
  cad_percept::cpt_deviation_analysis::DeviationMeshPublisher node(nh, nh_private);

  ros::spin();

  return 0;
}
