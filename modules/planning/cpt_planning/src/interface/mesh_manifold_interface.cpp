#include <cpt_planning/interface/mesh_manifold_interface.h>
#include <glog/logging.h>

namespace cad_percept {
namespace planning {

Eigen::Matrix3d MeshManifoldInterface::J(const Eigen::Vector3d x) const {
  // look up triangle
  Eigen::Vector2d x_uv = x.topRows<2>();
  auto faces = mapping_.nearestFace(x_uv);

  // get jacobian of 3d wrt uv
  return faces.second.getJacobianWrt(faces.first);
}

MeshManifoldInterface::StateX MeshManifoldInterface::convertToX(const StateQ &state_q) const {
  StateX uvh_state;
  uvh_state.pos_ = mapping_.point3DtoUVH(state_q.pos_);
  uvh_state.vel_ = J(uvh_state.pos_) * state_q.vel_;
  return uvh_state;
}

MeshManifoldInterface::StateQ MeshManifoldInterface::convertToQ(const StateX &state_x) const {
  LOG(WARNING) << "MeshManifoldInterface::convertToQ not implemented";
  return StateQ();
}

Eigen::Matrix3d MeshManifoldInterface::J(const StateX &state) const { return J(state.pos_); }

}  // namespace planning
}  // namespace cad_percept