#include <cpt_planning_eval/ompl/ompl_mesh_projecting_planner.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/PlannerDataGraph.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>

namespace cad_percept {
namespace planning {

void MeshProjectionConstraints::function(const Eigen::Ref<const Eigen::VectorXd> &x,
                                         Eigen::Ref<Eigen::VectorXd> out) const {
  // squared distance to surface
  out[0] = model_->squaredDistance(cad_percept::cgal::Point(x.x(), x.y(), x.z()));
}

void MeshProjectionConstraints::jacobian(const Eigen::Ref<const Eigen::VectorXd> &x,
                                         Eigen::Ref<Eigen::MatrixXd> out) const {
  // Get closest point on surface
  auto meshpoint = model_->getClosestTriangle(x.x(), x.y(), x.z());
  Eigen::Vector3d point(x.x(), x.y(), x.z());
  Eigen::Vector3d intersection(meshpoint.first.x(), meshpoint.first.y(), meshpoint.first.z());

  // get direction from current position (x) to the intersection
  Eigen::Vector3d direction = point - intersection;

  // fill in jacobian
  out.leftCols<3>(0) = direction.transpose();
}

OMPLMeshProjectingPlanner::OMPLMeshProjectingPlanner(std::string meshpath, double time)
    : solve_time_(time) {
  cad_percept::cgal::MeshModel::create(meshpath, &model_, true);
}

const cad_percept::planning::SurfacePlanner::Result OMPLMeshProjectingPlanner::plan(
    const Eigen::Vector3d start, const Eigen::Vector3d goal,
    std::vector<Eigen::Vector3d> *states_out) {
  ob::StateSpacePtr space(new ob::RealVectorStateSpace(3));

  // get bounding box of mesh
  auto bbox = model_->getBounds();
  ob::RealVectorBounds bounds(3);
  for (int i = 0; i < 3; i++) {
    bounds.setLow(i, bbox.min_coord(i));
    bounds.setHigh(i, bbox.max_coord(i));
  }
  // set up sampler to sample in bounded R^3
  space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

  // set up projection constraints
  auto constraint = std::make_shared<MeshProjectionConstraints>(model_);
  auto css = std::make_shared<ob::ProjectedStateSpace>(space, constraint);
  auto csi = std::make_shared<ob::ConstrainedSpaceInformation>(css);
  auto ss = std::make_shared<og::SimpleSetup>(csi);

  // set up planner using space defined above
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

  // Start solving
  std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
  ob::PlannerStatus solved = ss->solve(ob::timedPlannerTerminationCondition(solve_time_));

  bool reached_criteria = false;
  if (solved == ob::PlannerStatus::EXACT_SOLUTION) {
    // Succesfully connected!
    reached_criteria = true;

    // interpolate onto surface using constraints.
    ompl::geometric::PathGeometric path = ss->getSolutionPath();
    path.interpolate();

    // get result as trajectory
    for (auto state : path.getStates()) {
      Eigen::Vector3d pt;
      pt = *state->as<ob::ConstrainedStateSpace::StateType>();

      states_out->push_back(pt);
    }
  }
  std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();

  // Read out all intermediate states from the tree
  ob::PlannerData intermediate_states(csi);
  optimizingPlanner->getPlannerData(intermediate_states);
  auto graph = intermediate_states.toBoostGraph();
  boost::graph_traits<ob::PlannerData::Graph>::edge_iterator ei, ei_end;
  typedef boost::graph_traits<ob::PlannerData::Graph>::vertex_descriptor Vertex;
  boost::property_map<ob::PlannerData::Graph::Type, vertex_type_t>::type vertices =
      get(vertex_type_t(), graph);

  // and save them in an easier to read format.
  rrt_tree_.clear();
  for (boost::tie(ei, ei_end) = edges(graph); ei != ei_end; ++ei) {
    auto source_vtx = vertices[source(*ei, graph)];
    auto target_vtx = vertices[target(*ei, graph)];
    Eigen::Vector3d data_source_vtx =
        *source_vtx->getState()->as<ob::ConstrainedStateSpace::StateType>();
    Eigen::Vector3d data_target_vtx =
        *target_vtx->getState()->as<ob::ConstrainedStateSpace::StateType>();

    // populate tree of edges
    rrt_tree_.push_back({data_source_vtx, data_target_vtx});
  }

  SurfacePlanner::Result result;
  result.success = reached_criteria;
  result.duration = end_time - start_time;
  return result;
}
}  // namespace planning
}  // namespace cad_percept