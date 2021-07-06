#include <cpt_planning_ros/omav_planner.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "omav_planner_node");
  // ros::NodeHandle nh;

  cad_percept::planning::OMAVPlanner node(/*nh*/);

  ros::spin();

  return 0;
}
