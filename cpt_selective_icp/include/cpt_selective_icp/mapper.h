#ifndef SELECTIVE_ICP_MAPPER_H_
#define SELECTIVE_ICP_MAPPER_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "cpt_selective_icp/mapper_parameters.h"


namespace cad_percept {
namespace mapper {

typedef PointMatcher<float> PM;

class Mapper {
  public:
    Mapper(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
    ~Mapper();

  private:
  ros::NodeHandle &nh_, &nh_private_;
  MapperParameters parameters_;
  ros::Subscriber cloud_sub_;

  int odom_received_;
  tf::TransformListener tf_listener_;

  PM::TransformationParameters T_scanner_to_map_;


  /**
   * Load parameters for libpointmatcher from yaml
   */
  void loadExternalParameters();
  bool reloadallYaml(std_srvs::Empty::Request &req,
                     std_srvs::Empty::Response &res);

  // Parameters
  int min_reading_point_count;


};

}
}

#endif // SELECTIVE_ICP_MAPPER_H_