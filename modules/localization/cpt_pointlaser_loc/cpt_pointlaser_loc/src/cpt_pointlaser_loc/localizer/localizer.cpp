#include "cpt_pointlaser_loc/localizer/localizer.h"

#include "cpt_pointlaser_loc/optimizer/optimizer.h"

namespace cad_percept {
namespace pointlaser_loc {
namespace localizer {
PointLaserLocalizer::PointLaserLocalizer(
    const cad_percept::cgal::MeshModel::Ptr& model,
    const Eigen::Matrix<double, 6, 1>& initial_pose_std,
    const Eigen::Matrix<double, 6, 1>& odometry_noise_std,
    double pointlaser_noise_std)
    : model_(model),
      initial_pose_std_(initial_pose_std),
      odometry_noise_std_(odometry_noise_std),
      pointlaser_noise_std_(pointlaser_noise_std) {}

}  // namespace localizer
}  // namespace pointlaser_loc
}  // namespace cad_percept