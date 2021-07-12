#ifndef CPT_PLANNING_INTERFACE_LINEAR_MANIFOLD_INTERFACE_H_
#define CPT_PLANNING_INTERFACE_LINEAR_MANIFOLD_INTERFACE_H_

#include <cpt_planning/interface/manifold_interface.h>
#include <rmpcpp/core/geometry_base.h>

namespace cad_percept {
namespace planning {

class LinearManifoldInterface : public ManifoldInterface, public rmpcpp::GeometryBase<3, 3> {
 public:
  LinearManifoldInterface(){}
  virtual Eigen::Matrix3d J(const Eigen::Vector3d p_manifold) const;
  using StateX = rmpcpp::State<3>;
  using StateQ = rmpcpp::State<3>;

  virtual StateX convertToX(const StateQ &state_q) const;
  virtual StateQ convertToQ(const StateX &state_x) const;

 protected:
  virtual Eigen::Matrix3d J(const StateX &state) const;

};

}  // namespace planning
}  // namespace cad_percept
#endif  // CPT_PLANNING_INTERFACE_LINEAR_MANIFOLD_INTERFACE_H_
