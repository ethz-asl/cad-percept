//
// Created by mpantic on 13.12.20.
//

#include <cpt_chomp_planning/chomp_mesh_planner.h>
#include <cpt_ros/mesh_model_publisher.h>

#include <chrono>
#include <iostream>
int main(int argc, char *argv[]) {
  std::string scenario_curve =
      "/home/mpantic/Work/RAL_Manifolds/"
      "curve.off";

  ros::init(argc, argv, "chomp_test_node");

  chomp::ChompParameters params;
  //params.lambda = 500;
  params.lambda = 1500;
  //params.w_smooth = 0.01;
  params.w_collision = 10;
  params.w_smooth = 0.01;
  params.rel_tol = 1e-4;
  params.max_iter = 1000;
  params.epsilon = 0.5;
  params.map_resolution = 0.01;
  params.verbose = true;
  ChompMeshPlanner planner(scenario_curve, params);

  Eigen::Vector3d start, end;
  // start << -6.07483, -7.2267, 0.07645;
  start << 9.68184, -16.633, 0.118314;

  end << -5.0000, 17.2687, -10.2763;
  //end << 10.375, -4.94, -4.758;
  //end << 17.095, -1.19368, -0.87536;

  std::vector<Eigen::Vector3d> states_out;
  ros::NodeHandle nh;

  cad_percept::MeshModelPublisher pub_model(nh, "mesh");

  cad_percept::cgal::MeshModel::Ptr model;
  cad_percept::cgal::MeshModel::create(scenario_curve, &model, true);
  pub_model.publish(model);
  std::cin.get();
  planner.plan(start, end, &states_out);
}