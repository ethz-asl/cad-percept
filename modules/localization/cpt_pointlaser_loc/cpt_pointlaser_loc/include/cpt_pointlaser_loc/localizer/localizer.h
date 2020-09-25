#ifndef CPT_POINTLASER_LOC_LOCALIZER_LOCALIZER_H_
#define CPT_POINTLASER_LOC_LOCALIZER_LOCALIZER_H_

#include <cgal_definitions/mesh_model.h>

#include <Eigen/Geometry>

namespace cad_percept {
namespace pointlaser_loc {
namespace localizer {

/// \brief Simple class to localize the laser sensor w.r.t. a mesh model.
class PointLaserLocalizer {
 public:
  /// \brief Construct a new PointLaserLocalizer object.
  ///
  /// \param model                The mesh model w.r.t. which localization
  ///                             should be performed.
  /// \param initial_pose_std     Standard deviation of the initial pose.
  /// \param odometry_noise_std   Standard deviation of the odometry.
  /// TODO(fmilano): Check, the `odometry_noise_std` is not used internally by
  ///               optimizer.
  /// \param pointlaser_noise_std Standard deviation of the point-laser noise.
  PointLaserLocalizer(const cad_percept::cgal::MeshModel::Ptr& model,
                      const Eigen::Matrix<double, 6, 1>& initial_pose_std,
                      const Eigen::Matrix<double, 6, 1>& odometry_noise_std,
                      double pointlaser_noise_std);

 private:
  // Mesh model w.r.t. which localization should be performed.
  cad_percept::cgal::MeshModel::Ptr model_;
  // Noise statistics.
  Eigen::Matrix<double, 6, 1> initial_pose_std_, odometry_noise_std_;
  double pointlaser_noise_std_;
};
}  // namespace localizer
}  // namespace pointlaser_loc
}  // namespace cad_percept

#endif  // CPT_POINTLASER_LOC_LOCALIZER_LOCALIZER_H_