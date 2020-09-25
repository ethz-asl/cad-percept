#ifndef CPT_POINTLASER_LOC_OPTIMIZER_COMMON_H_
#define CPT_POINTLASER_LOC_OPTIMIZER_COMMON_H_

#include <gtsam/nonlinear/Expression.h>
#include <kindr/minimal/quat-transformation-gtsam.h>
#include <kindr/minimal/quat-transformation.h>

#include <Eigen/Geometry>

namespace cad_percept {
namespace pointlaser_loc {
namespace optimizer {

typedef gtsam::Expression<kindr::minimal::QuatTransformation> ETransformation;
typedef gtsam::Expression<Eigen::Vector3d> EVector3;

}  // namespace optimizer
}  // namespace pointlaser_loc
}  // namespace cad_percept

#endif  // CPT_POINTLASER_LOC_OPTIMIZER_COMMON_H_