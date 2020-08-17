#include <glog/logging.h>
#include <gtest/gtest.h>

#include <ros/ros.h>

/// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);

  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  return RUN_ALL_TESTS();
}
