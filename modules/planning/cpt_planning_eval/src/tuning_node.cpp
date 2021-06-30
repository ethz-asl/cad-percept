#include <cpt_planning_eval/PlannerTuningConfig.h>
#include <cpt_planning_eval/evaluation_node.h>
#include <cpt_planning_eval/ompl_mesh_projecting_planner.h>
#include <dynamic_reconfigure/server.h>

#include <chrono>
#include <iostream>

std::vector<cad_percept::planning::RMPMeshPlanner *> planners;
cad_percept::planning::EvaluationNode *evaluationNode;

void callback(cpt_planning_eval::PlannerTuningConfig &config, uint32_t level) {
  for (auto planner : planners) {
    planner->setTuning({config.a1, config.a2, config.a3}, {config.b1, config.b2, config.b3},
                       config.dt);
  }
  evaluationNode->plan(config.trigger_random, config.write, {9.63312, 5.56065, 7},
                       {9.37443, 2.55775, 3.22736});
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "ompl_test_node");
  ros::NodeHandle node_handle;
  std::string mesh_path = argv[1];

  evaluationNode = new cad_percept::planning::EvaluationNode(node_handle, mesh_path);
  double debug_factor = 3;

  cad_percept::planning::RMPMeshPlanner *rmp_planner =
      new cad_percept::planning::RMPMeshPlanner(mesh_path, {0.6, 5.8, 0.56}, {6.0, 8.0, 0.14});
  planners.push_back(rmp_planner);
  evaluationNode->addPlanner(rmp_planner, {1.0, 0.0, 0.0});

  evaluationNode->plan(true);


  //dynamic_reconfigure::Server<cpt_planning_eval::PlannerTuningConfig> server;
  //server.setCallback(boost::bind(&callback, _1, _2));

  //ros::spin();
  return 0;
}