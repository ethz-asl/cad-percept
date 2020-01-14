#ifndef CPT_PLANNING_INTERFACE_MESH_MANIFOLD_INTERFACE_H_
#define CPT_PLANNING_INTERFACE_MESH_MANIFOLD_INTERFACE_H_
#include <cgal_definitions/mesh_model.h>
#include <cpt_planning/interface/manifold_interface.h>

namespace cad_percept {
namespace planning {

class MeshManifoldInterface : public ManifoldInterface {
 public:
  MeshManifoldInterface(cgal::MeshModel::Ptr model) : model_(model) {}

  virtual Eigen::Matrix3d J(const Eigen::Vector3d p_manifold);

 private:
  cgal::MeshModel::Ptr model_;
};
}  // namespace planning
}  // namespace cad_percept
#endif  // CPT_PLANNING_INTERFACE_MESH_MANIFOLD_INTERFACE_H_
