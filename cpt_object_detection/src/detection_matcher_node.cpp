#include "cpt_object_detection/detection_matcher.h"
#include <ros/ros.h>

// Main function supporting the Mapper class
int main(int argc, char **argv) {
  ros::init(argc, argv, "detection_matcher");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  cad_percept::object_detection::DetectionMatcher mapper(nh, nh_private);
  ros::spin();

  return 0;
}
