#include <ros/ros.h>
#include "static_room_deviations/static_room_deviations_ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "static_room_deviations_node");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  std::shared_ptr<cad_percept::room_deviations::StaticRoomDeviations> static_room_deviations_ros_node;

  // Hand over the handle to the object
  static_room_deviations_ros_node = std::make_shared<cad_percept::room_deviations::StaticRoomDeviations>(nh, nh_private);

  ros::spin();

  return 0;
}
