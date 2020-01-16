#ifndef CPT_PLANNING_INTERFACE_MESH_MANIFOLD_INTERFACE_H_
#define CPT_PLANNING_INTERFACE_MESH_MANIFOLD_INTERFACE_H_
#include <cgal_definitions/mesh_model.h>
#include <cpt_planning/coordinates/uv_mapping.h>
#include <cpt_planning/interface/manifold_interface.h>

#include <rmpflow/core/geometry_base.h>

namespace cad_percept {
namespace planning {

 class MeshManifoldInterface : public ManifoldInterface, public rmp_core::GeometryBase<3,3> {
 public:
  MeshManifoldInterface(cgal::MeshModel::Ptr model, Eigen::Vector3d zero_point)
      : model_(model), mapping_(model, zero_point) {
  }
  virtual Eigen::Matrix3d J(const Eigen::Vector3d p_manifold);

  protected:
  virtual Eigen::Matrix3d J(const Eigen::Vector3d& p_manifold, const Eigen::Vector3d& p_dot);

 private:
  cgal::MeshModel::Ptr model_;
  UVMapping mapping_;
};
}  // namespace planning
}  // namespace cad_percept
#endif  // CPT_PLANNING_INTERFACE_MESH_MANIFOLD_INTERFACE_H_
