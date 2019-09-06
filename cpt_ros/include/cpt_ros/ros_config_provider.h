#ifndef CPT_ROS_ROS_CONFIG_PROVIDER_H_
#define CPT_ROS_ROS_CONFIG_PROVIDER_H_
#include <cgal_definitions/config_provider.h>
#include <ros/ros.h>
namespace cad_percept {

class RosConfigProvider : public ConfigProvider<std::string> {
 public:
  explicit RosConfigProvider(ros::NodeHandle nh) : nh_(nh) {}

  bool hasParam(std::string name) override;

  template <class T>
  T getParam(std::string name, T& default_value);

  ros::NodeHandle nh_;
};
}  // namespace cad_percept
#endif  // CPT_ROS_ROS_CONFIG_PROVIDER_H_
