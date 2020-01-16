#include <cpt_planning/interface/mesh_manifold_interface.h>

namespace cad_percept {
namespace planning {

Eigen::Matrix3d MeshManifoldInterface::J(const Eigen::Vector3d p_manifold) {

  // look up triangle
  Eigen::Vector2d p_manifold_uv = p_manifold.topRows<2>();
  auto faces = mapping_.nearestFace(p_manifold_uv);

  // get jacobian of 3d wrt uv
  return faces.second.getJacobianWrt(faces.first);
}

Eigen::Matrix3d MeshManifoldInterface::J(const Eigen::Vector3d& p_manifold, const Eigen::Vector3d& p_dot){
  return J(p_manifold);
}

}  // namespace planning
}  // namespace cad_percept