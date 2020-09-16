#ifndef CPT_ROS_ROS_CONFIG_PROVIDER_H_
#define CPT_ROS_ROS_CONFIG_PROVIDER_H_
#include <cgal_definitions/config_provider.h>
#include <ros/ros.h>
namespace cad_percept {

class RosConfigProvider : public ConfigProvider {
 public:
  explicit RosConfigProvider(ros::NodeHandle nh) : nh_(nh) {}

  bool hasParam(std::string name) override;

  GETTER_FOR_TYPE(int)
  GETTER_FOR_TYPE(double)
  GETTER_FOR_TYPE(bool)
  GETTER_FOR_TYPE(std::string)

 private:
  template <class T>
  T param_Impl(std::string name, const T& default_value);

  template <class T>
  bool getParam_Impl(std::string name, T& value);

  ros::NodeHandle nh_;
};
}  // namespace cad_percept
#endif  // CPT_ROS_ROS_CONFIG_PROVIDER_H_
