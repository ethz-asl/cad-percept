#include <cpt_planning_eval/PlannerTuningConfig.h>
#include <cpt_planning_eval/evaluation_node.h>
#include <cpt_planning_eval/ompl_mesh_projecting_planner.h>
#include <dynamic_reconfigure/server.h>

#include <chrono>
#include <iostream>

std::vector<cad_percept::planning::RMPMeshPlanner *> planners;
EvaluationNode *evaluationNode;

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "ompl_test_node");
  ros::NodeHandle node_handle;
  std::string mesh_path = argv[1];

  evaluationNode = new EvaluationNode(node_handle, mesh_path);

  Eigen::Vector3d tuning_1(0.7, 13.6, 0.4);
  Eigen::Vector3d tuning_2(20.0, 30.0, 0.01);
  double a1_min = 0.1, a1_max = 14;
  double b1_min = 0.1, b1_max = 20;

  cad_percept::planning::RMPMeshPlanner *rmp_planner =
      new cad_percept::planning::RMPMeshPlanner(mesh_path, tuning_1, tuning_2);
  planners.push_back(rmp_planner);
  evaluationNode->addPlanner(rmp_planner, {1.0, 0.0, 0.0});

  std::cout  << "TUNING "<<  tuning_1 << " " << tuning_2 << std::endl;
  evaluationNode->plan(false, true, {9.63312, 5.56065, 7}, {9.37443, 2.55775, 3.22736});

  std::cout << "a1 ranges" << std::endl;
  double a1 = a1_min;
  while (a1 < a1_max) {
    a1 = a1 * 2;
    Eigen::Vector3d newtuning_1(tuning_1);
    newtuning_1.x() = a1;
    std::cout  << "TUNING "<<  newtuning_1 << " " << tuning_2 << std::endl;
    rmp_planner->setTuning(newtuning_1, tuning_2);
    evaluationNode->plan(false, true, {9.63312, 5.56065, 7}, {9.37443, 2.55775, 3.22736});

  }
  std::cout << "b1 ranges" << std::endl;
  double b1 = b1_min;
  while (b1 < b1_max) {
    b1 = b1 * 2;
    Eigen::Vector3d newtuning_2(tuning_2);
    newtuning_2.x() = b1;
    std::cout  << "TUNING "<<  tuning_1 << " " << newtuning_2 << std::endl;

    rmp_planner->setTuning(tuning_1, newtuning_2);
    evaluationNode->plan(false, true, {9.63312, 5.56065, 7}, {9.37443, 2.55775, 3.22736});

  }

  return 0;
}