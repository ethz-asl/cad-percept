#include <cpt_planning/interface/linear_manifold_interface.h>
#include <glog/logging.h>

namespace cad_percept {
namespace planning {

Eigen::Matrix3d LinearManifoldInterface::J(const Eigen::Vector3d x) const {
  /**
   * Return jacobian. As the spaces are equal, this
   * is always identity.
   */
  return Eigen::Matrix3d::Identity();
}

LinearManifoldInterface::StateX LinearManifoldInterface::convertToX(const StateQ &state_q) const {
  return state_q;
}

LinearManifoldInterface::StateQ LinearManifoldInterface::convertToQ(const StateX &state_x) const {
  return state_x;
}

Eigen::Matrix3d LinearManifoldInterface::J(const StateX &state) const { 
    return J(state.pos_); 
}

}  // namespace planning
}  // namespace cad_percept