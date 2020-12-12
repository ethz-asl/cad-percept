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

void RMPMeshPlanner::storeMapping() const { mapping_->storeMapping(); }

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
  auto mesh = model_->getMeshPtr();
  int file_written = 0;
  integrator.resetTo(start_xyz);
  for (double t = 0; t < 500.0; t += dt_) {
    Eigen::Vector3d current_pos;

    current_pos = integrator.forwardIntegrate(policies, manifold_, dt_);

    Eigen::Vector3d bli, bla, blub;
    integrator.getState(&bli, &bla, &blub);
    states_out->push_back(current_pos);

    RMPG::StateQ current_stateQ{bli, bla};
    RMPG::StateX current_stateX = manifold_->convertToX(current_stateQ);
    // write out accelerations for each point in 2d
    // iterate through vertices

    std::ofstream file3d, file2d, filepos;
    file3d.open("/tmp/rmp/3d/" + std::to_string(file_written) + ".csv");
    file2d.open("/tmp/rmp/2d/" + std::to_string(file_written) + ".csv");
    filepos.open("/tmp/rmp/pos/" + std::to_string(file_written) + ".csv");

    for (cgal::Polyhedron::Vertex_iterator vtx = mesh->vertices_begin();
         vtx != mesh->vertices_end(); ++vtx) {
      // get 3d point
      cgal::Point point3d = vtx->point();
      Eigen::Vector3d point3deigen(point3d.x(), point3d.y(), point3d.z());

      // get 2d point
      cgal::Point_2 uv_point = mapping_->vertex_map_[vtx];
      Eigen::Vector3d point2deigen(uv_point.x(), uv_point.y(), 0.0);


      std::vector<typename TargetPolicy::PValue> evaluated_policies;
      for (TargetPolicy *policy : policies) {
        evaluated_policies.push_back(manifold_->at(current_stateX).pull(*policy));
      }
      Eigen::Vector3d acc_3d = TargetPolicy::PValue::sum(evaluated_policies).f_;

      std::vector<typename TargetPolicy::PValue> evaluated_policies_2d;

      for (TargetPolicy *policy : policies) {
        evaluated_policies_2d.push_back(policy->evaluateAt(current_stateX));
      }
      Eigen::Vector3d acc_2d = TargetPolicy::PValue::sum(evaluated_policies_2d).f_;

      file2d << point2deigen.x() << "\t" << point2deigen.y() << "\t" << point2deigen.z() << "\t"
             << acc_2d.x() << "\t" << acc_2d.y() << "\t" << acc_2d.z() << std::endl;

      file3d << point3d.x() << "\t" << point3d.y() << "\t" << point3d.z() << "\t"
             << acc_3d.x() << "\t" << acc_3d.y() << "\t" << acc_3d.z() << std::endl;

    }
    filepos  << current_pos.x() << "\t"  << current_pos.y() << "\t" << current_pos.z() << "\t"
             << current_stateX.pos_.x() << "\t"  << current_stateX.pos_.y() << "\t" << current_stateX.pos_.z() << std::endl;
    file2d.close();
    file3d.close();
    filepos.close();
    file_written++;

    if (integrator.isDone()) {
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