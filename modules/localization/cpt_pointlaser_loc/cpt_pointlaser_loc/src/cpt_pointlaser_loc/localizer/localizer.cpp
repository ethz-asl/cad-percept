#include "cpt_pointlaser_loc/localizer/localizer.h"


namespace cad_percept {
namespace pointlaser_loc {
namespace localizer {
PointLaserLocalizer::PointLaserLocalizer(const cad_percept::cgal::MeshModel::Ptr& model,
                                         const Eigen::Matrix<double, 6, 1>& initial_pose_std,
                                         const Eigen::Matrix<double, 6, 1>& odometry_noise_std,
                                         double pointlaser_noise_std)
    : model_(model),
      initial_pose_std_(initial_pose_std),
      odometry_noise_std_(odometry_noise_std),
      pointlaser_noise_std_(pointlaser_noise_std) {}

bool PointLaserLocalizer::setUpOptimizer(
    const kindr::minimal::QuatTransformation& marker_to_armbase,
    const kindr::minimal::QuatTransformation& initial_pose,
    const kindr::minimal::QuatTransformation& laser_a_offset,
    const kindr::minimal::QuatTransformation& laser_b_offset,
    const kindr::minimal::QuatTransformation& laser_c_offset,
    const kindr::minimal::QuatTransformation& endeffector_offset,
    const kindr::minimal::QuatTransformation& arm_base_to_base, bool fix_cad_planes,
    bool add_initial_pose_prior, bool only_optimize_translation) {
  // NOTE: we assume that the arm was already moved to the initial pose.

  // Instantiate an optimizer with the initial poses.
  optimizer_.reset(new cad_percept::pointlaser_loc::optimizer::LocalizationOptimizer(
      marker_to_armbase, initial_pose, model_, initial_pose_std_, odometry_noise_std_,
      pointlaser_noise_std_, fix_cad_planes, add_initial_pose_prior, only_optimize_translation));

  // Store the initial pose, required when adding measurements.
  initial_pose_.reset(new kindr::minimal::QuatTransformation(initial_pose));
}

}  // namespace localizer
}  // namespace pointlaser_loc
}  // namespace cad_percept