#include <cpt_reconstruction/reconstruction_classification.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "reconstruction_classification_node");
  ros::NodeHandle nodeHandle1;
  ros::NodeHandle nodeHandle2;
  cad_percept::cpt_reconstruction::Classification classification(nodeHandle1,
                                                                 nodeHandle2);
  return 0;
}
