#include <ros/ros.h>
#include "relative_deviations/relative_deviations_ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "srd_relative_deviations_node");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  std::shared_ptr<cad_percept::deviations::RelativeDeviations> srd_relative_deviations_ros_node;

  // Hand over the handle to the object
  srd_relative_deviations_ros_node = std::make_shared<cad_percept::deviations::RelativeDeviations>(nh, nh_private);

  ros::spin();

  return 0;
}
