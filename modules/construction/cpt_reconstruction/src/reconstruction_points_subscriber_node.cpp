#include <cpt_reconstruction/reconstruction_points_subscriber.h>
#include <cpt_reconstruction/reconstruction_preprocess_model.h>

#include <sstream>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "reconstruction_subscriber_node");
  ros::NodeHandle nodeHandle1;
  ros::NodeHandle nodeHandle2;

  std::string model_path =
      "/home/philipp/Schreibtisch/"
      "cla_c_vf_20150815_2020_demo_5m.ply";
  Eigen::Matrix4d transformation;
  transformation.setIdentity();
  cad_percept::cpt_reconstruction::PreprocessModel model(model_path,
                                                         transformation);
  model.preprocess();

  cad_percept::cpt_reconstruction::ReconstructionPointsSubscriber subscriber(
      nodeHandle1, nodeHandle2, &model);

  return 0;
}