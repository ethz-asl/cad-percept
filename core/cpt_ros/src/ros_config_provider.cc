#include <cpt_ros/ros_config_provider.h>
namespace cad_percept {

bool RosConfigProvider::hasParam(const std::string name) { return nh_.hasParam(name); }

template <class T>
T RosConfigProvider::param_Impl(const std::string name, const T& default_value) {
  return nh_.param(name, default_value);
}

template <class T>
bool RosConfigProvider::getParam_Impl(const std::string name, T& value) {
  return nh_.getParam(name, value);
}

void test() {
  ros::NodeHandle test;
  RosConfigProvider roscfg(test);
}
}  // namespace cad_percept
