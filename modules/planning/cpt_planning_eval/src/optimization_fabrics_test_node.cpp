// Dummy cc file that includes headers to make code completion work.
/*
*/

#include <Eigen/Dense>
#include <iostream>

#include <cpt_planning/implementation/rmp_linear_planner.h>

std::string waypoint_topic;


int main(int argc, char *argv[]) {
  ros::init(argc, argv, "optimization_fabrics_test_node");
  ros::NodeHandle nh("~");
  cad_percept::planning::RMPLinearPlanner rmp_planner;

  // read parameters 
  nh.param("waypoint_topic", waypoint_topic, std::string("/moving_target"));

  Eigen::Vector3d start_node_pos(-4., 0.75 , 0.);
  Eigen::Vector3d end_node_pos(1., 0.75, 0.);
  //the start node is fixed
  nh.param("rope_start_x", start_node_pos(0), -4.0);
  nh.param("rope_start_y", start_node_pos(1), 0.75);
  nh.param("rope_start_z", start_node_pos(2), 0.0);
  // the end is able to move 
  nh.param("rope_end_x", end_node_pos(0), 1.0);
  nh.param("rope_end_y", end_node_pos(1), 0.75);
  nh.param("rope_end_z", end_node_pos(2), 0.0);
  //init the middel drone
  Eigen::Vector3d init_drone_pos;
  init_drone_pos = 0.5*(start_node_pos + end_node_pos);
  rmp_planner.resetIntegrator(init_drone_pos, {0.,0.,0.});
  //init the rope model 
  rmp_planner.setTuning({0.7, 13.6, 0.4}, {20.0, 30.0, 0.01}, 
    start_node_pos, end_node_pos, 0.01);
  ros::Subscriber moving_target_sub = nh.subscribe(waypoint_topic, 10, 
    &cad_percept::planning::RMPLinearPlanner::goalCallback, &rmp_planner);

  rmp_planner.init_ros_interface(nh);

  rmp_planner.init_obs_wall();

  ros::spin();

  return 0;
}

