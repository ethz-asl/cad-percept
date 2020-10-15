#include <cpt_ompl_planning/evaluation_node.h>
#include <cpt_ompl_planning/ompl_mesh_projecting_planner.h>

#include <chrono>
#include <iostream>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "ompl_test_node");
  ros::NodeHandle node_handle;

  std::string mesh_path = argv[2];
  int num_trials = std::stoi(argv[1]);

  EvaluationNode evaluationNode(node_handle, mesh_path);

  for (int i = 0; i < 12; i++) {
    cad_percept::planning::RMPMeshPlanner *rmp_planner = new cad_percept::planning::RMPMeshPlanner(
        mesh_path, {0.6, 5.8, 0.56}, {6.0, 8.0, 0.14}, i);
    evaluationNode.addPlanner(rmp_planner, {i / 12.0, 0.0, 0.0});
  }

  cad_percept::planning::GeodesicMeshPlanner dgeo_planner(mesh_path);
  evaluationNode.addPlanner(&dgeo_planner, {0.0, 1.0, 0.0});

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