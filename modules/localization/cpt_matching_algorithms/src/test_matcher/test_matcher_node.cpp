#include <ros/ros.h>

#include "test_matcher/test_matcher.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "matcher_test");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  cad_percept::matching_algorithms::TestMatcher test_matcher(nh, nh_private);
  ros::spin();

  return 0;
}