#ifndef CPT_POINTLASER_COMM_DISTOBOX_SERIAL_H_
#define CPT_POINTLASER_COMM_DISTOBOX_SERIAL_H_

#include <serial/serial.h>

namespace cad_percept {
namespace pointlaser_comm {

class Distobox {
 public:
  Distobox(const std::string &port, const unsigned int num_sensors);
  ~Distobox();
  bool getDistance(uint32_t *distance_a, uint32_t *distance_b, uint32_t *distance_c);
  bool laserOn();
  bool laserOff();

 private:
  std::string sendCommand(unsigned int sensor, std::string command);
  bool sendCommand(unsigned int sensor, std::string command, std::string expectedAnswer);

  serial::Serial distobox_;
  unsigned int num_sensors_;
};
}  // namespace pointlaser_comm
}  // namespace cad_percept
#endif