// Dummy cc file that includes headers to make code completion work.
/*
*/
// #include <rmpcpp/core/policy_value.h>
// #include <rmpcpp/geometry/cylindrical_geometry.h>
// #include <rmpcpp/geometry/linear_geometry.h>
// #include <rmpcpp/policies/simple_target_policy.h>
// #include <rmpcpp/eval/trapezoidal_integrator.h>

#include <Eigen/Dense>
#include <iostream>

#include <plotty/matplotlibcpp.hpp>
#include <chrono>
// #include <cpt_planning/interface/surface_planner.h>
// #include <cpt_planning/interface/linear_manifold_interface.h>
#include <cpt_planning/implementation/rmp_linear_planner.h>

#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>


Eigen::Vector3d start{0.0, 0.0, 0.0};
Eigen::Vector3d goal_a{-0.1, -0.1, -0.1};
Eigen::Vector3d goal{10.0, -10.0, -2.0};
Eigen::Vector3d current_drone_pos{0.0, 0.0, 0.0};
nav_msgs::Path hose_path;
std::vector<Eigen::Vector3d> hose_key_points;

ros::Publisher drone_leader_pub;
ros::Publisher hose_path_pub;

nav_msgs::Path build_hose_model(std::vector<Eigen::Vector3d> &hose_key_points){
    // Circle parameters
    nav_msgs::Path waypoints;
    geometry_msgs::PoseStamped pt;

    waypoints.header.frame_id = std::string("enu");
    waypoints.header.stamp = ros::Time::now();
    pt.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    // pt.header.frame_id = std::string("enu");
    // pt.header.stamp = ros::Time::now();

    for(auto keypoint : hose_key_points){
      pt.pose.position.x =  keypoint(0);
      pt.pose.position.y =  keypoint(1);
      pt.pose.position.z =  keypoint(2);
      waypoints.poses.push_back(pt);    
    } 
    // Return
    return waypoints;
}

void timerCallback(const ros::TimerEvent&){
  //publish the drone_leader position
  geometry_msgs::PoseStamped pt;
  pt.header.frame_id = std::string("enu");
  pt.header.stamp = ros::Time::now();
  pt.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  pt.pose.position.x = goal(0);
  pt.pose.position.y = goal(1);
  pt.pose.position.z = goal(2);
  drone_leader_pub.publish(pt);

  hose_path_pub.publish(hose_path);
}

int main(int argc, char *argv[]) {
  //planner pipeline-----------------------------------------------------------------
  cad_percept::planning::RMPLinearPlanner rmp_planner;

  rmp_planner.setTuning({0.7, 13.6, 0.4}, {20.0, 30.0, 0.01}, 0.01);

  // Eigen::Vector3d goal{5.0, -10.0, 15.0};
  std::vector<Eigen::Vector3d> states_out;
  // rmp_planner.plan(start, goal, &states_out);

  // //plot-----------------------------------------------------------------------------
  // std::vector<double> plot_x, plot_y, plot_z;
  // for(Eigen::Vector3d i : states_out){
  //   // std::cout << i << std::endl;
  //   plot_x.push_back(i(0));
  //   plot_y.push_back(i(1));
  //   plot_z.push_back(i(2));
  //   //plotty::plot(i,"rx");
  // }
  // plotty::plot(plot_x,plot_y);
  // // plotty::plot(result.f_);
  // plotty::show();
  // //---------------------------------------------------------------------------------

  //ros visualization----------------------------------------------------------------
  ros::init(argc, argv, "optimization_fabrics_test_node");
  ros::NodeHandle nh;
  ros::Timer timer = nh.createTimer(ros::Duration(1.0), timerCallback);
  drone_leader_pub = nh.advertise<geometry_msgs::PointStamped>("drone_leader", 1000);
  hose_path_pub = nh.advertise<nav_msgs::Path>("hose_path", 1000);

  // ros::NodeHandle nh_private("~");
  rmp_planner.init_ros_interface(nh);
  mav_msgs::EigenTrajectoryPoint::Vector trajectory_odom;
  // test acc based potential
  // rmp_planner.generateTrajectoryOdom_2(start, goal, &trajectory_odom);
  //test balance policy:
  rmp_planner.generateTrajectoryOdom_3(start, goal_a, goal, &trajectory_odom);

  rmp_planner.publishTrajectory(trajectory_odom);

  //visualize the hose:
  current_drone_pos = trajectory_odom.back().position_W;
  std::vector<Eigen::Vector3d> hose_key_points;
  hose_key_points.push_back(goal_a);
  hose_key_points.push_back(current_drone_pos);
  hose_key_points.push_back(goal);
  hose_path = build_hose_model(hose_key_points);


  ros::spin();

  return 0;
}

