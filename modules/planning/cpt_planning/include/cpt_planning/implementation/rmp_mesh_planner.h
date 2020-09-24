#ifndef CPT_PLANNING_IMPLEMENTATION_RMP_MESH_PLANNER_H
#define CPT_PLANNING_IMPLEMENTATION_RMP_MESH_PLANNER_H
#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_definitions/mesh_model.h>
#include <cpt_planning/coordinates/face_coords.h>
#include <cpt_planning/coordinates/uv_mapping.h>
#include <cpt_planning/interface/mesh_manifold_interface.h>
#include <cpt_planning/interface/surface_planner.h>
#include <rmpcpp/core/space.h>
#include <rmpcpp/core/state.h>
#include <rmpcpp/eval/trapezoidal_integrator.h>
#include <rmpcpp/policies/simple_target_policy.h>

#include <iostream>
namespace cad_percept {
namespace planning {
class RMPMeshPlanner : SurfacePlanner {
 public:
  RMPMeshPlanner(std::string mesh_path);

  const SurfacePlanner::Result plan(const Eigen::Vector3d start, const Eigen::Vector3d goal,
                                    std::vector<Eigen::Vector3d> *states_out);

  inline const std::string getName() const { return "RMP"; }

  cad_percept::cgal::MeshModel::Ptr model_;
  cad_percept::planning::UVMapping *mapping_;
  cad_percept::planning::MeshManifoldInterface *manifold_;
};
}  // namespace planning
}  // namespace cad_percept

#endif  // CPT_PLANNING_IMPLEMENTATION_RMP_MESH_PLANNER_H
