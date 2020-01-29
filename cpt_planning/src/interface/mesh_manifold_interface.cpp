#include <cpt_planning/interface/mesh_manifold_interface.h>

namespace cad_percept {
namespace planning {

Eigen::Matrix3d MeshManifoldInterface::J(const Eigen::Vector3d x) const {

  // look up triangle
  Eigen::Vector2d x_uv = x.topRows<2>();
  auto faces = mapping_.nearestFace(x_uv);

  // get jacobian of 3d wrt uv
  return faces.second.getJacobianWrt(faces.first);
}

void MeshManifoldInterface::convertPosition(const Eigen::Vector3d &q,
                                            const Eigen::Vector3d &q_dot,
                                            Eigen::Vector3d *x,
                                            Eigen::Vector3d *x_dot) const {

  *x = mapping_.point3DtoUVH(q);
  *x_dot = J(*x) * q_dot;
}

Eigen::Matrix3d MeshManifoldInterface::J(const Eigen::Vector3d &x, const Eigen::Vector3d &x_dot) const {
  return J(x);
}

}  // namespace planning
}  // namespace cad_percept