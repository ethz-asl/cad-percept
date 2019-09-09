#include <cpt_ros/ros_config_provider.h>
namespace cad_percept {

bool RosConfigProvider::hasParam(const std::string name) { return nh_.hasParam(name); }

template <class T>
T RosConfigProvider::getParamImpl(std::string name, const T& default_value) {
  return nh_.param(name, default_value);
}

}  // namespace cad_percept
