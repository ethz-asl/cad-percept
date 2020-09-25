#ifndef CPT_POINTLASER_LOC_OPTIMIZER_OPTIMIZER_H_
#define CPT_POINTLASER_LOC_OPTIMIZER_OPTIMIZER_H_

#include <cgal_definitions/mesh_model.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <kindr/minimal/quat-transformation.h>

#include <Eigen/Geometry>

#include "cpt_pointlaser_loc/optimizer/common.h"

namespace cad_percept {
namespace pointlaser_loc {
namespace optimizer {

class LocalizationOptimizer {
 public:
  LocalizationOptimizer(
      const kindr::minimal::QuatTransformation& architecture_offset,
      const kindr::minimal::QuatTransformation& initial_pose,
      const cad_percept::cgal::MeshModel::Ptr& arch_model,
      const Eigen::Matrix<double, 6, 1>& architecture_offset_std,
      const Eigen::Matrix<double, 6, 1>& odometry_noise_std,
      const double pointlaser_noise_std, const bool fix_retrieved_planes,
      const bool add_prior, const bool only_optimize_translation);
  Eigen::Vector3d addRelativeMeasurement(
      const double distance,
      const kindr::minimal::QuatTransformation joint2sensor);
  void addOdometry(const kindr::minimal::QuatTransformation odometry);
  kindr::minimal::QuatTransformation optimize(const bool verbose = false);

 private:
  bool fix_retrieved_planes_, only_optimize_translation_;
  gtsam::ExpressionFactorGraph graph_;
  kindr::minimal::QuatTransformation current_arm_pose_,
      initial_architect_offset_;
  ETransformation architect_offset_;
  cad_percept::cgal::MeshModel::Ptr architect_model_;
  gtsam::noiseModel::Base::shared_ptr pointlaser_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr odometry_noise_;
  gtsam::Values initialization_;
};

}  // namespace optimizer
}  // namespace pointlaser_loc
}  // namespace cad_percept
#endif  // CPT_POINTLASER_LOC_OPTIMIZER_OPTIMIZER_H_
