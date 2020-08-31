#include <Eigen/Dense>

#ifndef CPT_PLANNING_INTERFACE_MANIFOLD_H_
#define CPT_PLANNING_INTERFACE_MANIFOLD_H_

namespace cad_percept {
namespace planning {

class ManifoldInterface {
 public:
  // p_manifold is defined as {u,v,z}
  virtual Eigen::Matrix3d J(const Eigen::Vector3d p_manifold) const = 0;
};
}  // namespace planning
}  // namespace cad_percept

#endif  // CPT_PLANNING_INTERFACE_MANIFOLD_H_
