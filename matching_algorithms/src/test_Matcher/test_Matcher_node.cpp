#include <ros/ros.h>

#include "test_Matcher/test_Matcher.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "matcher_test");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  // ROS_INFO_STREAM("Node is intialized");
  cad_percept::matching_algorithms::test_Matcher test_Matcher(nh, nh_private);
  // ROS_INFO_STREAM("Matcher is running")
  ros::spin();

  return 0;
}