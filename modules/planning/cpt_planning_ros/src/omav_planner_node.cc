#include <cpt_planning_ros/omav_planner.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "omav_planner_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  cad_percept::planning::OMAVPlanner node(nh, nh_private);

  ros::spin();

  return 0;
}
