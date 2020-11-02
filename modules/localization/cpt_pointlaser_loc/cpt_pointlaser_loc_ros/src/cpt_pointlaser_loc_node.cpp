#include "cpt_pointlaser_loc_ros/kinova_localizer.h"
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "pointlaser_loc");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  std::shared_ptr<cad_percept::pointlaser_loc_ros::KinovaLocalizer> auto_node;

  auto_node = std::make_shared<cad_percept::pointlaser_loc_ros::KinovaLocalizer>(nh, nh_private);

  ros::spin();
  return 0;
}