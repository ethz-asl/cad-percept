#include <cpt_ompl_planning/ompl_mesh_sampling_planner.h>

OMPLMeshSamplingPlanner::OMPLMeshSamplingPlanner(std::string meshpath, bool connect)
    : rrt_connect_(connect) {
  cad_percept::cgal::MeshModel::create(meshpath, &model_, true);
}

const cad_percept::planning::SurfacePlanner::Result OMPLMeshSamplingPlanner::plan(
    const Eigen::Vector3d start, const Eigen::Vector3d goal,
    std::vector<Eigen::Vector3d>* states_out) {
  ob::StateSpacePtr space(new ob::RealVectorStateSpace(3));
  space->as<ob::RealVectorStateSpace>()->setBounds(-23.0, 23.0);

  ob::SpaceInformationPtr si(new ob::SpaceInformation(space));

  si->setStateValidityChecker(
      std::bind(&OMPLMeshSamplingPlanner::meshStateValidityChecker, this, std::placeholders::_1));
  si->setStateValidityCheckingResolution(0.01);  // 1%

  auto ss = std::make_shared<og::SimpleSetup>(si);
  space->setStateSamplerAllocator(
      std::bind(&OMPLMeshSamplingPlanner::allocMeshManifoldSampler, this, std::placeholders::_1));

  ob::PlannerPtr optimizingPlanner;
  if (rrt_connect_) {
    optimizingPlanner.reset(new og::RRTConnect(si));
  } else {
    optimizingPlanner.reset(new og::RRTstar(si));
  }

  optimizingPlanner->setup();

  ss->setPlanner(optimizingPlanner);

  // set start & goal
  ob::ScopedState<> startstate(space);
  startstate->as<ob::RealVectorStateSpace::StateType>()->values[0] = start.x();
  startstate->as<ob::RealVectorStateSpace::StateType>()->values[1] = start.y();
  startstate->as<ob::RealVectorStateSpace::StateType>()->values[2] = start.z();

  ob::ScopedState<> goalstate(space);
  goalstate->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal.x();
  goalstate->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal.y();
  goalstate->as<ob::RealVectorStateSpace::StateType>()->values[2] = goal.z();

  ss->setStartAndGoalStates(startstate, goalstate);

  std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
  ob::PlannerStatus solved = ss->solve(solve_time_);
  bool reached_criteria = false;

  if (solved == ob::PlannerStatus::EXACT_SOLUTION) {
    reached_criteria = true;
    // get path
    ompl::geometric::PathGeometric path = ss->getSolutionPath();

    for (auto state : path.getStates()) {
      Eigen::Vector3d pt;
      pt.x() = state->as<ob::RealVectorStateSpace::StateType>()->values[0];
      pt.y() = state->as<ob::RealVectorStateSpace::StateType>()->values[1];
      pt.z() = state->as<ob::RealVectorStateSpace::StateType>()->values[2];
      states_out->push_back(pt);
    }
  }

  std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();

  SurfacePlanner::Result result;
  result.success = reached_criteria;
  result.duration = end_time - start_time;
  return result;
}