#include <cpt_reconstruction/reconstruction_model.h>
#include <cpt_reconstruction/reconstruction_shape_detection.h>

//#include <sstream>
//#include <string>
//#include "ros/ros.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "reconstruction_shape_detection_node");
  ros::NodeHandle nodeHandle1;
  ros::NodeHandle nodeHandle2;

  cad_percept::cpt_reconstruction::Model model(nodeHandle1);
  model.preprocess();

  cad_percept::cpt_reconstruction::ShapeDetection subscriber(
      nodeHandle1, nodeHandle2, &model);

  return 0;
}