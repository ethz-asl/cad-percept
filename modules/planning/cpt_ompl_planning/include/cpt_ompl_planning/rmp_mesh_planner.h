//
// Created by mpantic on 23.09.20.
//

#ifndef CPT_OMPL_PLANNING_RMP_MESH_PLANNER_H
#define CPT_OMPL_PLANNING_RMP_MESH_PLANNER_H
#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_definitions/mesh_model.h>
#include <cpt_planning/coordinates/face_coords.h>
#include <cpt_planning/coordinates/uv_mapping.h>
#include <cpt_planning/interface/mesh_manifold_interface.h>
#include <rmpcpp/core/space.h>
#include <rmpcpp/core/state.h>
#include <rmpcpp/eval/trapezoidal_integrator.h>
#include <rmpcpp/policies/simple_target_policy.h>

#include <iostream>

class RMPMeshPlanner {
 public:
  RMPMeshPlanner() {
    std::cout << cad_percept::cgal::MeshModel::create(
                     "/home/mpantic/ws/rmp/src/manifold_simulations/models/hilo_roof/meshes/"
                     "hilo_reconstructed.off",
                     &model_, true)
              << std::endl;
    Eigen::Vector3d zero(0.0, 0.0, 0.0);
    double zero_angle = 0;
    mapping_ = new cad_percept::planning::UVMapping(model_, zero, zero_angle);
    manifold_ = new cad_percept::planning::MeshManifoldInterface(model_, zero, zero_angle);
    ros::NodeHandle nh;

  }

  bool plan(Eigen::Vector3d startpoint, Eigen::Vector3d endpoint,
            std::vector<Eigen::Vector3d> *pathpoints) {
    using RMPG = cad_percept::planning::MeshManifoldInterface;
    using LinSpace = rmpcpp::Space<3>;
    using TargetPolicy = rmpcpp::SimpleTargetPolicy<LinSpace>;
    using Integrator = rmpcpp::TrapezoidalIntegrator<TargetPolicy, RMPG>;

    RMPG::VectorQ target_xyz = endpoint;
    RMPG::VectorX target_uv = mapping_->point3DtoUVH(target_xyz);

    RMPG::VectorQ start_xyz = startpoint;
    RMPG::VectorX start_uv = mapping_->point3DtoUVH(start_xyz);
    Integrator integrator;

    Eigen::Matrix3d A{Eigen::Matrix3d::Identity()};
    Eigen::Matrix3d B{Eigen::Matrix3d::Identity()};
    A.diagonal() = Eigen::Vector3d({1.0, 1.0, 0.0});
    B.diagonal() = Eigen::Vector3d({0.0, 0.0, 1.0});
    TargetPolicy pol2(target_uv, A, 0.6, 5.8, 0.56);  // goes to manifold as quick as possible
    TargetPolicy pol3(Eigen::Vector3d::Zero(), B, 6.0, 8.0, 0.14);  // stays along it
    std::vector<TargetPolicy *> policies;
    policies.push_back(&pol2);
    policies.push_back(&pol3);

    integrator.resetTo(start_xyz);
    double dt = 0.01;
    for (double t = 0; t < 35.0; t += dt) {
      Eigen::Vector3d current_pos;
      current_pos = integrator.forwardIntegrate(policies, manifold_, dt);

      pathpoints->push_back(current_pos);

      if ((current_pos - target_xyz).norm() < 0.01) {
        std::cout << "bla" << std::endl;
        ROS_WARN_STREAM("Integrator finished after " << t << " s with a distance of "
                                                     << integrator.totalDistance());
        return true;
        break;
      }
    }
    return false;
  }

  cad_percept::cgal::MeshModel::Ptr model_;
  cad_percept::planning::UVMapping *mapping_;
  cad_percept::planning::MeshManifoldInterface *manifold_;
};

#endif  // CPT_OMPL_PLANNING_RMP_MESH_PLANNER_H
