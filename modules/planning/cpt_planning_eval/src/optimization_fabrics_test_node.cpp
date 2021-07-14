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



int main(int argc, char *argv[]) {
  //planner pipeline-----------------------------------------------------------------
  cad_percept::planning::RMPLinearPlanner rmp_planner;

  rmp_planner.setTuning({0.7, 13.6, 0.4}, {20.0, 30.0, 0.01}, 0.01);

  Eigen::Vector3d start{0.0, 0.0, 0.0};
  Eigen::Vector3d goal{5.0, 5.0, 0.0};
  std::vector<Eigen::Vector3d> states_out;
  rmp_planner.plan(start, goal, &states_out);

  //plot-----------------------------------------------------------------------------
  std::vector<double> plot_x, plot_y, plot_z;
  for(Eigen::Vector3d i : states_out){
    // std::cout << i << std::endl;
    plot_x.push_back(i(0));
    plot_y.push_back(i(1));
    plot_z.push_back(i(2));
    //plotty::plot(i,"rx");
  }
  plotty::plot(plot_x,plot_y);
  // plotty::plot(result.f_);
  plotty::show();
  //---------------------------------------------------------------------------------
  //ros visualization----------------------------------------------------------------
  ros::init(argc, argv, "optimization_fabrics_test_node");
  ros::NodeHandle nh;
  // ros::NodeHandle nh_private("~");
  rmp_planner.init_ros_interface(nh);
  mav_msgs::EigenTrajectoryPoint::Vector trajectory_odom;
  rmp_planner.generateTrajectoryOdom(start, goal, &trajectory_odom);
  rmp_planner.publishTrajectory(trajectory_odom);
  ros::spin();

  //---------------------------------------------------------------------------------
  return 0;
}
