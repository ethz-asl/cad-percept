#include <cpt_utils/altitude_publisher.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "altitude_publisher_node");
  // Creating the node handles
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Create Node object

  cad_percept::cpt_utils::AltitudePublisher node(nh, nh_private);

  ros::spin();

  return 0;
}