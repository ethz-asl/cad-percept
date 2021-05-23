#include <cpt_reconstruction/reconstruction_clustering.h>

//#include <sstream>
//#include <string>
//#include "ros/ros.h"
//#include "std_msgs/String.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "reconstruction_clustering_node");
  ros::NodeHandle nodeHandle1;
  ros::NodeHandle nodeHandle2;
  cad_percept::cpt_reconstruction::Clustering clustering(nodeHandle1,
                                                         nodeHandle2);

  return 0;
}
