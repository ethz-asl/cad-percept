#include <cpt_ompl_planning/ompl_mesh_projecting_planner.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>

OMPLMeshProjectingPlanner::OMPLMeshProjectingPlanner(std::string meshpath, double time)
    : solve_time_(time) {
  cad_percept::cgal::MeshModel::create(meshpath, &model_, true);
}

const cad_percept::planning::SurfacePlanner::Result OMPLMeshProjectingPlanner::plan(
    const Eigen::Vector3d start, const Eigen::Vector3d goal,
    std::vector<Eigen::Vector3d> *states_out) {
  ob::StateSpacePtr space(new ob::RealVectorStateSpace(3));
  space->as<ob::RealVectorStateSpace>()->setBounds(-23.0, 23.0);

  auto constraint = std::make_shared<MeshConstraints>(model_);
  auto css = std::make_shared<ob::ProjectedStateSpace>(space, constraint);
  auto csi = std::make_shared<ob::ConstrainedSpaceInformation>(css);

  auto ss = std::make_shared<og::SimpleSetup>(csi);
  ob::PlannerPtr optimizingPlanner;

  optimizingPlanner.reset(new og::RRTstar(csi));
  optimizingPlanner->setup();

  ss->setPlanner(optimizingPlanner);
  // set start & goal
  ob::ScopedState<> startstate(css);
  startstate->as<ob::ConstrainedStateSpace::StateType>()->copy(start);

  ob::ScopedState<> goalstate(css);
  goalstate->as<ob::ConstrainedStateSpace::StateType>()->copy(goal);

  ss->setStartAndGoalStates(startstate, goalstate);

  std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
  ob::PlannerStatus solved = ss->solve(solve_time_);
  bool reached_criteria = false;

  if (solved == ob::PlannerStatus::EXACT_SOLUTION) {
    reached_criteria = true;
    // get path
    ompl::geometric::PathGeometric path = ss->getSolutionPath();
    path.interpolate();
    for (auto state : path.getStates()) {
      Eigen::Vector3d pt;
      pt = *state->as<ob::ConstrainedStateSpace::StateType>();

      states_out->push_back(pt);
    }
  }

  std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();

  SurfacePlanner::Result result;
  result.success = reached_criteria;
  result.duration = end_time - start_time;
  return result;
}