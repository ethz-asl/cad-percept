#include <cpt_reconstruction/reconstruction_points_publisher.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "reconstruction_points_publisher_node");
  ros::NodeHandle nodeHandle;
  cad_percept::cpt_reconstruction::ReconstructionPointsPublisher publisher(
      nodeHandle);

  return 0;
}