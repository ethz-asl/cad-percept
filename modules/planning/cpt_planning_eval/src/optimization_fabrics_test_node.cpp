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
  for(Eigen::Vector3d i : states_out){
    std::cout << i << std::endl;
    //plotty::plot(i,"rx");
  }
  // plotty::plot(result.f_);
  //plotty::show();

  return 0;
}
