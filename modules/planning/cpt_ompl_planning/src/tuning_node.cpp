#include <cpt_ompl_planning/PlannerTuningConfig.h>
#include <cpt_ompl_planning/evaluation_node.h>
#include <cpt_ompl_planning/ompl_mesh_projecting_planner.h>
#include <dynamic_reconfigure/server.h>

#include <chrono>
#include <iostream>

cad_percept::planning::RMPMeshPlanner *rmp_planner;
EvaluationNode *evaluationNode;

void callback(cpt_ompl_planning::PlannerTuningConfig &config, uint32_t level) {
  rmp_planner->setTuning({config.a1, config.a2, config.a3}, {config.b1, config.b2, config.b3}, config.dt);

  evaluationNode->plan(config.trigger_random);
}

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

  evaluationNode = new EvaluationNode(node_handle, mesh_path);
  double debug_factor = 3;
  rmp_planner = new cad_percept::planning::RMPMeshPlanner(mesh_path);

  evaluationNode->addPlanner(rmp_planner, {1.0, 0.0, 0.0});
  evaluationNode->plan(true);

  dynamic_reconfigure::Server<cpt_ompl_planning::PlannerTuningConfig> server;
  server.setCallback(boost::bind(&callback, _1, _2));

  ros::spin();
  return 0;
}