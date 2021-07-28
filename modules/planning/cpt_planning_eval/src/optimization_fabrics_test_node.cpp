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

// #include <plotty/matplotlibcpp.hpp>
// #include <chrono>
// #include <cpt_planning/interface/surface_planner.h>
// #include <cpt_planning/interface/linear_manifold_interface.h>
#include <cpt_planning/implementation/rmp_linear_planner.h>

// #include <tf/tf.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <nav_msgs/Path.h>
// #include <visualization_msgs/MarkerArray.h>
// #include <visualization_msgs/Marker.h>


// Eigen::Vector3d start{0.0, 0.0, 0.0};
// Eigen::Vector3d goal_a{-0.1, -0.1, -0.1};
// Eigen::Vector3d goal{0.0, 10.0, 2.0};
// Eigen::Vector3d current_drone_pos{0.0, 0.0, 0.0};
// nav_msgs::Path hose_path;
// std::vector<Eigen::Vector3d> hose_key_points;
// std::vector<Eigen::Vector3d> simple_obs_wall;

// ros::Publisher drone_leader_pub;
// ros::Publisher hose_path_pub;
// ros::Publisher obs_vis_pub;


// void init_obs_wall(std::vector<Eigen::Vector3d> &simple_obs_wall){
//   double y=5.0;
//   for(double x = -1.5; x<1.5; x=x+0.5){
//     for(double z = 0.; z<1.5; z=z+0.5){
//       Eigen::Vector3d point{x,y,z};
//       simple_obs_wall.push_back(point);
//     }
//   }
// }

// void publish_obs_vis(std::vector<Eigen::Vector3d> &simple_obs_wall) {
//     visualization_msgs::MarkerArray obsArray;
//     auto obs_frame = std::string("enu");
//     auto obs_time = ros::Time::now();

//     int id = 0;
//     for (auto obs_point : simple_obs_wall) {
//         visualization_msgs::Marker p;
//         p.type = visualization_msgs::Marker::SPHERE;
//         p.id = id;
//         p.header.frame_id = obs_frame;
//         p.header.stamp = obs_time;
//         p.scale.x = 0.1;
//         p.scale.y = 0.1;
//         p.scale.z = 0.1;
//         p.pose.orientation.w = 1.0;
//         p.pose.position.x=obs_point(0);
//         p.pose.position.y=obs_point(1);
//         p.pose.position.z=obs_point(2);
//         p.color.a = 1.0;
//         p.color.r = 1.0;
//         p.color.g = 1.0;
//         p.color.b = 0.0;

//         obsArray.markers.push_back(p);
//         id++;
//     }
//     obs_vis_pub.publish(obsArray);
// }

// nav_msgs::Path build_hose_model(std::vector<Eigen::Vector3d> &hose_key_points){
//     // Circle parameters
//     nav_msgs::Path waypoints;
//     geometry_msgs::PoseStamped pt;

//     waypoints.header.frame_id = std::string("enu");
//     waypoints.header.stamp = ros::Time::now();
//     pt.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
//     // pt.header.frame_id = std::string("enu");
//     // pt.header.stamp = ros::Time::now();
//     for(auto keypoint : hose_key_points){
//       pt.pose.position.x =  keypoint(0);
//       pt.pose.position.y =  keypoint(1);
//       pt.pose.position.z =  keypoint(2);
//       waypoints.poses.push_back(pt);    
//     } 
//     // Return
//     return waypoints;
// }

// void timerCallback(const ros::TimerEvent&){
//   //publish the drone_leader position
//   geometry_msgs::PoseStamped pt;
//   pt.header.frame_id = std::string("enu");
//   pt.header.stamp = ros::Time::now();
//   pt.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
//   pt.pose.position.x = goal(0);
//   pt.pose.position.y = goal(1);
//   pt.pose.position.z = goal(2);
//   drone_leader_pub.publish(pt);

//   // hose_path_pub.publish(hose_path);

//   //visualize the simple obs wall:
//   // publish_obs_vis(simple_obs_wall);
// }


int main(int argc, char *argv[]) {
  //planner pipeline-----------------------------------------------------------------
  cad_percept::planning::RMPLinearPlanner rmp_planner;

  rmp_planner.setTuning({0.7, 13.6, 0.4}, {20.0, 30.0, 0.01}, 0.01);

  // Eigen::Vector3d goal{5.0, -10.0, 15.0};
  // std::vector<Eigen::Vector3d> states_out;
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
  // ros::Timer timer = nh.createTimer(ros::Duration(3.0), timerCallback);
  // drone_leader_pub = nh.advertise<geometry_msgs::PointStamped>("drone_leader", 1000);
  // hose_path_pub = nh.advertise<nav_msgs::Path>("hose_path", 1000);
  // obs_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("obs_vis", 10);

  ros::Subscriber moving_target_sub = nh.subscribe("move_base_simple/goal", 10, 
                                    &cad_percept::planning::RMPLinearPlanner::goalCallback, &rmp_planner);


  // ros::NodeHandle nh_private("~");
  rmp_planner.init_ros_interface(nh);
  // mav_msgs::EigenTrajectoryPoint::Vector trajectory_odom;
  // test acc based potential
  // rmp_planner.generateTrajectoryOdom_2(start, goal, &trajectory_odom);
  //test balance policy:
  // rmp_planner.generateTrajectoryOdom_3(start, goal_a, goal, &trajectory_odom);
  //test obs avoid policy:
  // rmp_planner.init_obs_wall();
  // std::cout << " simple_obs_wall: " << simple_obs_wall.back() << std::endl;    
  // rmp_planner.generateTrajectoryOdom_4(start, goal_a, goal, 
  //                                     simple_obs_wall, &trajectory_odom);
  // test moving end-effector
  rmp_planner.init_obs_wall();
  // rmp_planner.generateTrajectoryOdom_5();
  // rmp_planner.publishTrajectory_2();

  ros::spin();

  return 0;
}

