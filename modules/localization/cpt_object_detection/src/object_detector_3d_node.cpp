#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>

#include "cpt_object_detection/object_detector_3d.h"

// Main function supporting the Mapper class
int main(int argc, char **argv) {
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  ros::init(argc, argv, "detection_matcher");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  cad_percept::object_detection::ObjectDetector3D mapper(nh, nh_private);
  ros::spin();

  return 0;
}
