#include <cpt_planning/interface/mesh_manifold_interface.h>

namespace cad_percept {
namespace planning {

Eigen::Matrix3d MeshManifoldInterface::J(const Eigen::Vector3d p_manifold) {

  // look up triangle
  auto faces = mapping_.nearestFace(p_manifold);

  // get jacobian of 3d wrt uv
  return faces.second.getJacobianWrt(faces.first);
}

}  // namespace planning
}  // namespace cad_percept