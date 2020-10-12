
#include <iostream>

#include "cpt_pointlaser_comm/distobox_serial.h"
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "pointlaser_comm");

  ros::NodeHandle nh_private("~");

  cad_percept::pointlaser_comm::Distobox distobox(
      nh_private.param<std::string>("port", ""),
      (unsigned int)nh_private.param<int>("num_sensors", 3));

  ros::ServiceServer distance = nh_private.advertiseService(
      "distance", &cad_percept::pointlaser_comm::Distobox::getDistance, &distobox);
  ros::ServiceServer laserOn = nh_private.advertiseService(
      "laserOn", &cad_percept::pointlaser_comm::Distobox::laserOn, &distobox);
  ros::ServiceServer laserOff = nh_private.advertiseService(
      "laserOff", &cad_percept::pointlaser_comm::Distobox::laserOff, &distobox);

  ros::spin();
  return 0;
}
