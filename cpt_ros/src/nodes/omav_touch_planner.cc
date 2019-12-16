#include <cpt_ros/nodes/omav_touch_planner.h>


int main(int argc, char** argv) {
  ros::init(argc, argv, "collision_manifold_test_node");
  // Creating the node handles
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Create Node object
  cad_percept::OmavTouchPlanner node(nh, nh_private);

  ros::spin();

  return 0;
}