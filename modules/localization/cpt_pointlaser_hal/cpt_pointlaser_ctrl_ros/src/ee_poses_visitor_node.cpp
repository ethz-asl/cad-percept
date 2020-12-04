#include "cpt_pointlaser_ctrl_ros/ee_poses_visitor.h"
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "pointlaser_loc");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  std::shared_ptr<cad_percept::pointlaser_ctrl_ros::EEPosesVisitor> auto_node;

  auto_node = std::make_shared<cad_percept::pointlaser_ctrl_ros::EEPosesVisitor>(nh, nh_private);

  ros::spin();
  return 0;
}