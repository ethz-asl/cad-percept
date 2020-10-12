#include "cpt_pointlaser_comm/distobox_serial.h"

#include <glog/logging.h>

/*
 * Documentation of possible commands for the leica distobox sensors:
 * https://drive.google.com/drive/folders/1-QAtW--5QD4X0LCJrSZk-tYrwnI2jf9i
 *
 * Most imporant commands
 * a - switches on sensor, laser off
 * b - switches off sensor
 * c - resets sensor to standby
 * e - show configuration data of sensor. did not work with our sensor.
 * o - laser on. did sometimes not work with our sensor.
 * t - measure temperature of sensor
 * G 0 - measure distance and switch laser off afterwards
 * G 1 - measure distance and leave laser on afterwards
 * K - measure signal
 * $ON - switches distobox on
 * $OFF - switches distobox off
 * $S<x> - switches to distobox <x> in multiplex !no whitespace!
 */

namespace cad_percept {
namespace pointlaser_comm {

Distobox::Distobox(const std::string &port, const unsigned int num_sensors)
    : distobox_(port, 115200, serial::Timeout::simpleTimeout(1000)), num_sensors_(num_sensors) {
  if (!distobox_.isOpen()) {
    LOG(WARNING) << "failed to open serial port" << std::endl;
  }
  for (unsigned int sensor = 0; sensor < num_sensors_; sensor++) {
    if (!sendCommand(sensor, "$ON\r\n", "?\r\n")) {
      LOG(WARNING) << "Error when switching on sensor " << sensor << std::endl;
    }
    // after switching on, the first command is always returning an error. We therefore simply run
    // some first commands.
    distobox_.flush();
    distobox_.write("a\r\n");
    distobox_.readline();
    distobox_.flush();
  }
}

Distobox::~Distobox() {
  for (unsigned int sensor = 0; sensor < num_sensors_; sensor++) {
    sendCommand(sensor, "$OFF\r\n");
  }
  distobox_.close();
}

std::string Distobox::sendCommand(unsigned int sensor, std::string command) {
  distobox_.flush();
  bool success = false;
  while (!success) {
    distobox_.write("$S" + std::to_string(sensor) + "\r\n");
    std::string switchAnswer = distobox_.readline();
    success = switchAnswer == "?\r\n";
    if (!success) LOG(WARNING) << "Distobox error: " << switchAnswer;
  }
  distobox_.flush();
  distobox_.write(command);
  std::string answer = distobox_.readline();
  distobox_.flush();
  return answer;
}

bool Distobox::sendCommand(unsigned int sensor, std::string command, std::string expectedAnswer) {
  std::string answer = sendCommand(sensor, command);
  if (answer != expectedAnswer) {
    LOG(WARNING) << "Error at Sensor " << sensor << ": " << answer;
    return false;
  }
  return true;
}

bool Distobox::getDistance(uint32_t *distance_a, uint32_t *distance_b, uint32_t *distance_c) {
  CHECK_NOTNULL(distance_a);
  CHECK_NOTNULL(distance_b);
  CHECK_NOTNULL(distance_c);
  std::vector<unsigned long> distances;
  for (unsigned int sensor = 0; sensor < num_sensors_; sensor++) {
    std::string measurement = sendCommand(sensor, "G 1\r\n");
    if (measurement.size() != 34) {
      // Any other format indicates an error
      LOG(WARNING) << "Sensor error when measuring at " << sensor << ": " << measurement;
      return false;
    }
    distances.push_back(stoul(measurement.substr(7, 8)));
  }
  *distance_a = distances[0];
  *distance_b = distances[1];
  *distance_c = distances[2];
  return true;
}

bool Distobox::laserOn() {
  for (unsigned int sensor = 0; sensor < num_sensors_; sensor++) {
    // In case that the laser device is off, the first 'o' will switch on the device and only the
    // second 'o' will switch on the laser.
    sendCommand(sensor, "o\r\n", "?\r\n");
    if (!sendCommand(sensor, "o\r\n", "?\r\n")) return false;
  }
  return true;
}

bool Distobox::laserOff() {
  for (unsigned int sensor = 0; sensor < num_sensors_; sensor++) {
    if (!sendCommand(sensor, "b\r\n", "?\r\n")) return false;
  }
  return true;
}
}  // namespace pointlaser_comm
}  // namespace cad_percept
