#include <ros/ros.h>

#include "cpt_selective_icp/mapper.h"

// Main function supporting the Mapper class
int main(int argc, char **argv) {
  ros::init(argc, argv, "mapper");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  cad_percept::mapper::Mapper mapper(nh, nh_private);
  ros::spin();

  return 0;
}