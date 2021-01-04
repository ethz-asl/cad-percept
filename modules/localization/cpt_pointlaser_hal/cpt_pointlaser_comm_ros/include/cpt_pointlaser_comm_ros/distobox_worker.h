#ifndef CPT_POINTLASER_COMM_ROS_DISTOBOX_WORKER_H_
#define CPT_POINTLASER_COMM_ROS_DISTOBOX_WORKER_H_

#include <cpt_pointlaser_comm/distobox_serial.h>
#include <std_srvs/Empty.h>

#include "cpt_pointlaser_comm_ros/GetDistance.h"

namespace cad_percept {
namespace pointlaser_comm_ros {

class DistoboxWorker {
 public:
  DistoboxWorker(const std::string &port, const unsigned int num_sensors);

  bool getDistance(cpt_pointlaser_comm_ros::GetDistance::Request &request,
                   cpt_pointlaser_comm_ros::GetDistance::Response &response);
  bool laserOn(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);
  bool laserOff(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);

 private:
  cad_percept::pointlaser_comm::Distobox distobox_;
};
}  // namespace pointlaser_comm_ros
}  // namespace cad_percept
#endif