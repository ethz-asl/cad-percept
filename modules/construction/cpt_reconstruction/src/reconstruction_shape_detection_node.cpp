#include <cpt_reconstruction/reconstruction_model.h>
#include <cpt_reconstruction/reconstruction_shape_detection.h>

//#include <sstream>
//#include <string>
//#include "ros/ros.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "reconstruction_shape_detection_node");
  ros::NodeHandle nodeHandle1;
  ros::NodeHandle nodeHandle2;

  std::string model_path =
      "/home/philipp/Schreibtisch/data/"
      "CLA_MissingParts_1_8m.ply";
  Eigen::Matrix4d transformation;
  transformation.setIdentity();
  cad_percept::cpt_reconstruction::Model model(model_path, transformation);
  model.preprocess();

  cad_percept::cpt_reconstruction::ShapeDetection subscriber(
      nodeHandle1, nodeHandle2, &model);

  return 0;
}