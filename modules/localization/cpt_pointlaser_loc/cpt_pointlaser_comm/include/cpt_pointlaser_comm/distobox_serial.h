#include "cpt_pointlaser_comm/GetDistance.h"
#include "serial/serial.h"
#include "std_srvs/Empty.h"

namespace cad_percept {
namespace pointlaser_comm {

class Distobox {
 public:
  Distobox(const std::string& port, const unsigned int num_sensors);
  ~Distobox();
  std::string sendCommand(unsigned int sensor, std::string command);
  bool sendCommand(unsigned int sensor, std::string command, std::string expectedAnswer);
  bool getDistance(cpt_pointlaser_comm::GetDistance::Request& request,
                   cpt_pointlaser_comm::GetDistance::Response& response);
  bool laserOn(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  bool laserOff(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

 private:
  serial::Serial distobox_;
  unsigned int num_sensors_;
};
}  // namespace pointlaser_comm
}  // namespace cad_percept
