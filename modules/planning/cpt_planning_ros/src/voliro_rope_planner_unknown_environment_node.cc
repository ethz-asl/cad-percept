// Dummy cc file that includes headers to make code completion work.
/*
*/
#include <iostream>
#include <cpt_planning_ros/voliro_rope_planner_unknown_environment.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "optimization_fabrics_test_node_unknown_environment");
  ros::NodeHandle nh("~");
  ros::NodeHandle nh_private("~");

  cad_percept::planning::VoliroRopePlanner voliro_rope_planner(nh, nh_private);
  ros::spin();

  return 0;
}

