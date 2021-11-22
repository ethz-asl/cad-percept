#include <cpt_planning_eval/chomp/chomp_mesh_planner.h>

const cad_percept::planning::SurfacePlanner::Result ChompMeshPlanner::plan(
    const Eigen::Vector3d start, const Eigen::Vector3d goal,
    std::vector<Eigen::Vector3d>* states_out) {
  cad_percept::planning::SurfacePlanner::Result result;
  chomp::ChompTrajectory traject;

  std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
  result.success = chomper_.solveProblem(start, goal, 150, &traject);
  std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
  result.duration = end_time - start_time;

  for (int i = 0; i < traject.trajectory.rows(); i++) {
    states_out->push_back(traject.trajectory.block<1, 3>(i, 0).transpose());
  }

  return result;
}

double ChompMeshPlanner::getMeshDistance(const Eigen::VectorXd& position) {
  cad_percept::cgal::Point pt(position[0], position[1], position[2]);
  double dist =  sqrt(model_->squaredDistance(pt));
  return dist;
}

ChompMeshPlanner::ChompMeshPlanner(std::string meshpath, chomp::ChompParameters params) {
  cad_percept::cgal::MeshModel::create(meshpath, &model_, true);
  chomper_.setup();
  chomper_.setParameters(params);
  chomper_.setDistanceFunction(
      std::bind(&ChompMeshPlanner::getMeshDistance, this, std::placeholders::_1));
}
