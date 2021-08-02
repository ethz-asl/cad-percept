// Dummy cc file that includes headers to make code completion work.
/*
*/

#include <Eigen/Dense>
#include <iostream>

#include <cpt_planning/implementation/rmp_linear_planner.h>

std::string waypoint_topic;


int main(int argc, char *argv[]) {
  cad_percept::planning::RMPLinearPlanner rmp_planner;

  rmp_planner.setTuning({0.7, 13.6, 0.4}, {20.0, 30.0, 0.01}, 0.01);

  ros::init(argc, argv, "optimization_fabrics_test_node");
  ros::NodeHandle nh("~");
  nh.param("waypoint_topic", waypoint_topic, std::string("/moving_target"));

  ros::Subscriber moving_target_sub = nh.subscribe(waypoint_topic, 10, 
                                    &cad_percept::planning::RMPLinearPlanner::goalCallback, &rmp_planner);

  rmp_planner.init_ros_interface(nh);

  rmp_planner.init_obs_wall();

  ros::spin();

  return 0;
}

