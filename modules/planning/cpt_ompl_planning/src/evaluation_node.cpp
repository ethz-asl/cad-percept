#include <cpt_ompl_planning/evaluation_node.h>
#include <cpt_ompl_planning/ompl_mesh_projecting_planner.h>

#include <cpt_chomp_planning/chomp_mesh_planner.h>
#include <chrono>
#include <iostream>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "ompl_test_node");
  ros::NodeHandle node_handle;

  std::string mesh_path = argv[2];
  int num_trials = std::stoi(argv[1]);

  std::string scenario_curve =
      "/home/mpantic/Work/ICRA_Manifolds/"
      "curve.off";

  std::string scenario_rhone =
      "/home/mpantic/ws/rmp/src/manifold_simulations/models/rhone_mesh/meshes/rhone_enu_0.1m.off";

  // mesh_path =
  // "/home/mpantic/ws/rmp/src/manifold_simulations/models/rhone_mesh/meshes/rhone_enu.off";
  /*std::string mesh_path =
      "/home/mpantic/ws/rmp/src/manifold_simulations/models/hilo_roof/meshes/hilo_reconstructed.off";*/

  EvaluationNode evaluationNode(node_handle, mesh_path);
  double debug_factor = 1;
  cad_percept::planning::RMPMeshPlanner rmp_planner(mesh_path);
  // rmp_planner.setTuning({1.0, 11.0, 0.81}, {8.8, 20.0, 0.06}, 0.01);
    //rmp_planner.setTuning({0.7, 11.6, 0.81}, {20.0, 30.0, 0.01}, 0.01);
  rmp_planner.setTuning({0.7, 13.6, 0.4}, {20.0, 30.0, 0.01}, 0.01);
  //rmp_planner.storeMapping();


  cad_percept::planning::GeodesicMeshPlanner dgeo_planner(mesh_path);
  OMPLMeshSamplingPlanner ompl_planner(mesh_path, false, 1.0 * debug_factor);
  OMPLMeshSamplingPlanner ompl_planner_fast(mesh_path, false, 0.25 * debug_factor);

  OMPLMeshSamplingPlanner ompl_connect_planner(mesh_path, true, 1.0 * debug_factor);

  OMPLMeshProjectingPlanner ompl_planner_projecting(mesh_path, 1.0 * debug_factor);
  OMPLMeshProjectingPlanner ompl_planner_projecting_fast(mesh_path, 0.25 * debug_factor);
  chomp::ChompParameters params;
  //params.lambda = 500;
  params.lambda = 1500;
  params.w_smooth = 0.01;
  params.w_collision = 10;

  params.rel_tol = 1e-6;
  params.max_iter = 1000;
  params.epsilon = 1.25;
  params.map_resolution = 0.01;
  params.verbose = false;
  ChompMeshPlanner chomp_planner(mesh_path, params);


  evaluationNode.addPlanner(&rmp_planner, {1.0, 0.0, 0.0});
  evaluationNode.addPlanner(&dgeo_planner, {0.0, 1.0, 0.0});

  evaluationNode.addPlanner(&ompl_planner, {0.0, 0.0, 1.0});
  evaluationNode.addPlanner(&ompl_connect_planner, {1.0, 0.0, 1.0});
  evaluationNode.addPlanner(&ompl_planner_fast, {0.0, 0.5, 1.0});

  evaluationNode.addPlanner(&ompl_planner_projecting, {0.0, 1.0, 1.0});
  evaluationNode.addPlanner(&ompl_planner_projecting_fast, {0.5, 1.0, 1.0});
  evaluationNode.addPlanner(&chomp_planner, {0.5, 1.0, 0.0});


  int i_trials = 0;

  while (ros::ok(), i_trials < num_trials) {
    std::cout << std::endl << std::endl << std::endl;
    std::cout << " ==================== Trial " << i_trials
              << " ====================  " << std::endl;
    std::cout << std::endl << std::endl << std::endl;
    evaluationNode.plan();
    i_trials++;
  }
  return 0;
}