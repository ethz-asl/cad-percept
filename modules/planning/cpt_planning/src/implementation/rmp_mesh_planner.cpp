#include <cpt_planning/implementation/rmp_mesh_planner.h>

namespace cad_percept {
namespace planning {

RMPMeshPlanner::RMPMeshPlanner(std::string mesh_path, Eigen::Vector3d tuning_1,
                               Eigen::Vector3d tuning_2, int mapping)
    : tuning_1_(tuning_1), tuning_2_(tuning_2), mapping_id_(mapping) {
  cad_percept::cgal::MeshModel::create(mesh_path, &model_, true);
  Eigen::Vector3d zero(0.0, 0.0, 0.0);
  double zero_angle = 0;
  mapping_ = new cad_percept::planning::UVMapping(model_, zero, zero_angle, mapping_id_);
  manifold_ = new cad_percept::planning::MeshManifoldInterface(model_, zero, zero_angle);
}

const SurfacePlanner::Result RMPMeshPlanner::plan(const Eigen::Vector3d start,
                                                  const Eigen::Vector3d goal,
                                                  std::vector<Eigen::Vector3d> *states_out) {
  // Set up solver
  using RMPG = cad_percept::planning::MeshManifoldInterface;
  using LinSpace = rmpcpp::Space<3>;
  using TargetPolicy = rmpcpp::SimpleTargetPolicy<LinSpace>;
  using Integrator = rmpcpp::TrapezoidalIntegrator<TargetPolicy, RMPG>;

  RMPG::VectorQ target_xyz = goal;
  RMPG::VectorX target_uv = mapping_->point3DtoUVH(target_xyz);

  RMPG::VectorQ start_xyz = start;
  RMPG::VectorX start_uv = mapping_->point3DtoUVH(start_xyz);
  Integrator integrator;

  // set up policies
  Eigen::Matrix3d A{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d B{Eigen::Matrix3d::Identity()};
  A.diagonal() = Eigen::Vector3d({1.0, 1.0, 0.0});
  B.diagonal() = Eigen::Vector3d({0.0, 0.0, 1.0});

  TargetPolicy pol2(target_uv, A, tuning_1_[0], tuning_1_[1],
                    tuning_1_[2]);  // goes to target
  TargetPolicy pol3(Eigen::Vector3d::Zero(), B, tuning_2_[0], tuning_2_[1],
                    tuning_2_[2]);  // stays on surface
  std::vector<TargetPolicy *> policies;
  policies.push_back(&pol2);
  policies.push_back(&pol3);

  // start integrating path
  std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
  bool reached_criteria = false;

  integrator.resetTo(start_xyz);
  for (double t = 0; t < 500.0; t += dt_) {
    Eigen::Vector3d current_pos;
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

    current_pos = integrator.forwardIntegrate(policies, manifold_, dt_);

    std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
   /* std::cout << "Integrate TIME = "
              << std::chrono::duration_cast<std::chrono::microseconds>(
                  (end_time - start_time))
                  .count()
              << std::endl;*/
    states_out->push_back(current_pos);

    if(integrator.isDone()) {
      reached_criteria = true;
      std::cout << (current_pos - target_xyz).norm() << std::endl;

      break;
    }
    /*if ((current_pos - target_xyz).norm() < 0.005) {
      Eigen::Vector3d bli, bla, blub;
      integrator.getState(&bli, &bla, &blub);
      std::cout << bla << std::endl;
      std::cout << blub << std::endl;
      reached_criteria = true;
      break;
    }*/
  }
  std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();

  SurfacePlanner::Result result;
  result.success = reached_criteria;
  result.duration = end_time - start_time;
  return result;
}
}  // namespace planning
}  // namespace cad_percept