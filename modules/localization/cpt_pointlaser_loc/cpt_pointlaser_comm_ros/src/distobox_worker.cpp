#include "cpt_pointlaser_comm_ros/distobox_worker.h"

namespace cad_percept {
namespace pointlaser_comm_ros {

DistoboxWorker::DistoboxWorker(const std::string &port, const unsigned int num_sensors)
    : distobox_(port, num_sensors) {}

bool DistoboxWorker::getDistance(cpt_pointlaser_comm_ros::GetDistance::Request &request,
                                 cpt_pointlaser_comm_ros::GetDistance::Response &response) {
  return distobox_.getDistance(&response.distanceA, &response.distanceB, &response.distanceC);
}

bool DistoboxWorker::laserOn(std_srvs::Empty::Request &request,
                             std_srvs::Empty::Response &response) {
  return distobox_.laserOn();
}

bool DistoboxWorker::laserOff(std_srvs::Empty::Request &request,
                              std_srvs::Empty::Response &response) {
  return distobox_.laserOff();
}
}  // namespace pointlaser_comm_ros
}  // namespace cad_percept
