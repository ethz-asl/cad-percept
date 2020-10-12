#ifndef CPT_POINTLASER_LOC_LOCALIZER_LOCALIZER_H_
#define CPT_POINTLASER_LOC_LOCALIZER_LOCALIZER_H_

#include <cgal_definitions/mesh_model.h>
#include <kindr/minimal/position.h>
#include <kindr/minimal/quat-transformation.h>

#include <Eigen/Geometry>

#include "cpt_pointlaser_loc/optimizer/optimizer.h"

namespace cad_percept {
namespace pointlaser_loc {
namespace localizer {

/// \brief Simple class to localize the laser sensor w.r.t. a mesh model.
class PointLaserLocalizer {
 public:
  /// \brief Construct a new PointLaserLocalizer object.
  ///
  /// \param model                The mesh model w.r.t. which localization should be performed.
  /// \param initial_pose_std     Standard deviation of the initial pose.
  /// \param odometry_noise_std   Standard deviation of the odometry.
  /// TODO(fmilano): Check, the `odometry_noise_std` is not used internally by optimizer.
  /// \param pointlaser_noise_std Standard deviation of the point-laser noise.
  PointLaserLocalizer(const cad_percept::cgal::MeshModel::Ptr& model,
                      const Eigen::Matrix<double, 6, 1>& initial_pose_std,
                      const Eigen::Matrix<double, 6, 1>& odometry_noise_std,
                      double pointlaser_noise_std);

  /// \brief Performs localization w.r.t. the mesh model.
  /// NOTE: it is assumed that the arm was already moved to the initial pose.
  //  TODO(fmilano): Check for initial pose.
  /// TODO(fmilano): Check for exact meaning of poses.
  ///
  /// \param marker_to_armbase          Pose between the marker and the arm base.
  /// \param initial_pose               Pose between the arm base and the Kinova link.
  /// \param laser_a_offset             Pose between the Kinova link and the point laser A.
  /// \param laser_b_offset             Pose between the Kinova link and the point laser B.
  /// \param laser_c_offset             Pose between the Kinova link and the point laser C.
  /// \param endeffector_offset         Pose between the Kinova link and the Kinova end effector.
  /// \param arm_base_to_base           Pose between the arm base and the base.
  /// \param fix_cad_planes             If True, the planes retrieved by the optimizer are fixed.
  ///  \param add_initial_pose_prior    If True, a factor for the prior of the model offset is added
  ///                                   to the optimization graph.
  /// \param only_optimize_translation  If True, only optimizes translation.
  /// \return True if localization can be successfully performed, False otherwise.
  bool setUpOptimizer(const kindr::minimal::QuatTransformation& marker_to_armbase,
                      const kindr::minimal::QuatTransformation& initial_pose,
                      const kindr::minimal::QuatTransformation& laser_a_offset,
                      const kindr::minimal::QuatTransformation& laser_b_offset,
                      const kindr::minimal::QuatTransformation& laser_c_offset,
                      const kindr::minimal::QuatTransformation& endeffector_offset,
                      const kindr::minimal::QuatTransformation& arm_base_to_base,
                      bool fix_cad_planes = false, bool add_initial_pose_prior = false,
                      bool only_optimize_translation = false);

  /// \brief Updates and returns the goal pose of the arm, given an input movement.
  ///
  /// \param rotation_quat  Rotation component of the movement (quaternion format).
  /// \param translation    Translation component of the movement.
  ///
  /// \return Goal pose of the arm.
  kindr::minimal::QuatTransformation getArmGoalPose(
      Eigen::Quaternion<double> rotation_quat,
      kindr::minimal::PositionTemplate<double> translation);

  /// \brief Adds an odometry transform to the optimization.
  ///
  /// \param odometry_transform  Odometry transformation to add to the optimization.
  ///
  /// \return None.
  void addOdometry(const kindr::minimal::QuatTransformation &odometry_transform);

  /// \brief Adds laser measurements to the optimization.
  ///
  /// \param distance_A  Distance measured by laser A.
  /// \param distance_B  Distance measured by laser B.
  /// \param distance_C  Distance measured by laser C.
  ///
  /// \return None.
  void addLaserMeasurements(uint32_t distance_A, uint32_t distance_B, uint32_t distance_C);

 private:
  // Optimizer.
  std::unique_ptr<cad_percept::pointlaser_loc::optimizer::LocalizationOptimizer> optimizer_;
  // Mesh model w.r.t. which localization should be performed.
  cad_percept::cgal::MeshModel::Ptr model_;
  // Noise statistics.
  Eigen::Matrix<double, 6, 1> initial_pose_std_, odometry_noise_std_;
  double pointlaser_noise_std_;
  // Initial pose between arm base and Kinova link.
  std::unique_ptr<kindr::minimal::QuatTransformation> initial_pose_;
  // Offsets between the Kinova link and the lasers.
  std::unique_ptr<kindr::minimal::QuatTransformation> laser_a_offset_;
  std::unique_ptr<kindr::minimal::QuatTransformation> laser_b_offset_;
  std::unique_ptr<kindr::minimal::QuatTransformation> laser_c_offset_;
  // Goal pose of the arm.
  std::unique_ptr<kindr::minimal::QuatTransformation> arm_goal_pose_;
};
}  // namespace localizer
}  // namespace pointlaser_loc
}  // namespace cad_percept

#endif  // CPT_POINTLASER_LOC_LOCALIZER_LOCALIZER_H_