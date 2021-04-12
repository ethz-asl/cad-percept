#include <ros/ros.h>

#include "cpt_selective_icp_dyn/mapper.h"

// Main function supporting the Mapper class
int main(int argc, char **argv) {
  ros::init(argc, argv, "mapper");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  cad_percept::selective_icp_dyn::Mapper mapper(nh, nh_private);
  ros::spin();

  return 0;
}
