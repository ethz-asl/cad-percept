#include <cpt_ompl_planning/evaluation_node.h>
#include <cpt_ompl_planning/ompl_mesh_projecting_planner.h>

#include <chrono>
#include <iostream>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "ompl_test_node");
  ros::NodeHandle node_handle;
  /* std::string mesh_path =
       "/home/mpantic/Work/ICRA_Manifolds/"
       "curve.off";
 */
  std::string mesh_path =
      "/home/mpantic/ws/rmp/src/manifold_simulations/models/rhone_mesh/meshes/"
      "rhone_enu.off";

  EvaluationNode evaluationNode(node_handle, mesh_path);
  double debug_factor = 3;
  cad_percept::planning::RMPMeshPlanner rmp_planner(mesh_path);
  cad_percept::planning::GeodesicMeshPlanner dgeo_planner(mesh_path);
  OMPLMeshSamplingPlanner ompl_planner(mesh_path, false, 1.0 * debug_factor);
  OMPLMeshSamplingPlanner ompl_planner_fast(mesh_path, false, 0.25 * debug_factor);

  OMPLMeshSamplingPlanner ompl_connect_planner(mesh_path, true, 1.0 * debug_factor);

  OMPLMeshProjectingPlanner ompl_planner_projecting(mesh_path, 1.0 * debug_factor);
  OMPLMeshProjectingPlanner ompl_planner_projecting_fast(mesh_path, 0.25 * debug_factor);

  evaluationNode.addPlanner(&rmp_planner, {1.0, 0.0, 0.0});
  evaluationNode.addPlanner(&dgeo_planner, {0.0, 1.0, 0.0});

  evaluationNode.addPlanner(&ompl_planner, {0.0, 0.0, 1.0});
  evaluationNode.addPlanner(&ompl_connect_planner, {1.0, 0.0, 1.0});
  evaluationNode.addPlanner(&ompl_planner_fast, {0.0, 0.5, 1.0});

  evaluationNode.addPlanner(&ompl_planner_projecting, {0.0, 1.0, 1.0});
  evaluationNode.addPlanner(&ompl_planner_projecting_fast, {0.5, 1.0, 1.0});

  while (ros::ok()) {
    evaluationNode.plan();
    getchar();
  }
  return 0;
}