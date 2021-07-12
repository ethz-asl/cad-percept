#include <cpt_planning/implementation/rmp_linear_planner.h>
#include <glog/logging.h>

namespace cad_percept {
namespace planning {

RMPLinearPlanner::RMPLinearPlanner(Eigen::Vector3d tuning_1,
                               Eigen::Vector3d tuning_2)
    : tuning_1_(tuning_1), tuning_2_(tuning_2) {
//   cad_percept::cgal::MeshModel::create(mesh_path, &model_, true);
//   Eigen::Vector3d zero(0.0, 0.0, 0.0);
//   double zero_angle = 0;
//   mapping_ = new cad_percept::planning::UVMapping(model_, zero, zero_angle);
  manifold_ =
      std::make_shared<cad_percept::planning::LinearManifoldInterface>();
}

const SurfacePlanner::Result RMPLinearPlanner::plan(const Eigen::Vector3d start,
                                                  const Eigen::Vector3d goal,
                                                  std::vector<Eigen::Vector3d> *states_out) {
  // Set up solver
  using RMPG = cad_percept::planning::LinearManifoldInterface;
  using LinSpace = rmpcpp::Space<3>;
  using TargetPolicy = rmpcpp::SimpleTargetPolicy<LinSpace>;
  using Integrator = rmpcpp::TrapezoidalIntegrator<TargetPolicy, RMPG>;

  RMPG::VectorQ target_xyz = goal;
  RMPG::VectorX target_uv = target_xyz;

  RMPG::VectorQ start_xyz = start;
  RMPG::VectorX start_uv = start_xyz;
  Integrator integrator;

  // set up policies
  Eigen::Matrix3d A{Eigen::Matrix3d::Identity()};
//   Eigen::Matrix3d B{Eigen::Matrix3d::Identity()};
//   A.diagonal() = Eigen::Vector3d({1.0, 1.0, 0.0});
//   B.diagonal() = Eigen::Vector3d({0.0, 0.0, 1.0});

  auto pol_2 = std::make_shared<TargetPolicy>(target_uv, A, tuning_1_[0], tuning_1_[1],
                                              tuning_1_[2]);  // goes to target
//   auto pol_3 =
//       std::make_shared<TargetPolicy>(Eigen::Vector3d::Zero(), B, tuning_2_[0], tuning_2_[1],
//                                      tuning_2_[2]);  // stays on surface

  std::vector<std::shared_ptr<TargetPolicy>> policies;
  policies.push_back(pol_2);
//   policies.push_back(pol_3);

  // start integrating path
  std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
  bool reached_criteria = false;

  integrator.resetTo(start_xyz);
  for (double t = 0; t < 500.0; t += dt_) {
    auto integrator_state = integrator.integrateStep(policies, manifold_, dt_);
    states_out->push_back(integrator_state.position);

    if (integrator.atRest(0.01, 0.01)) {
      reached_criteria = true;
      LOG(INFO) << "Residual position after integration: "
                << (integrator_state.position - target_xyz).norm();
      break;
    }
  }
  std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();

  SurfacePlanner::Result result;
  result.success = reached_criteria;
  result.duration = end_time - start_time;
  return result;
}
}  // namespace planning
}  // namespace cad_percept