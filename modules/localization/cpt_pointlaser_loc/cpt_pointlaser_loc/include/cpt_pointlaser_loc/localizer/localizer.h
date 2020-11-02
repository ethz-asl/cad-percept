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

  /// \brief Sets up the optimizer for a new high-accuracy localization query.
  /// NOTE: it is assumed that the arm was already moved to the initial pose.
  //  TODO(fmilano): Check for initial pose.
  /// TODO(fmilano): Check for exact meaning of poses.
  ///
  /// \param marker_to_armbase          Pose between the marker and the arm base.
  /// \param initial_pose               Pose between the arm base and the arm link.
  /// \param laser_a_offset             Pose between the arm link and the point laser A.
  /// \param laser_b_offset             Pose between the arm link and the point laser B.
  /// \param laser_c_offset             Pose between the arm link and the point laser C.
  /// \param endeffector_offset         Pose between the arm link and the arm end effector.
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
  void addOdometry(const kindr::minimal::QuatTransformation& odometry_transform);

  /// \brief Adds laser measurements to the optimization.
  ///
  /// \param distance_a  Distance measured by laser A (in 1/10 mm).
  /// \param distance_b  Distance measured by laser B (in 1/10 mm).
  /// \param distance_c  Distance measured by laser C (in 1/10 mm).
  ///
  /// \return None.
  void addLaserMeasurements(uint32_t distance_a, uint32_t distance_b, uint32_t distance_c);

  /// \brief Returns the intersections of the 3 lasers with the model.
  ///
  /// \param[in] current_arm_pose  Current pose of the arm link w.r.t. the arm base.
  /// \param[out] intersection_a   Intersection of laser A with the model.
  /// \param[out] intersection_b   Intersection of laser B with the model.
  /// \param[out] intersection_c   Intersection of laser C with the model.
  ///
  /// \return None.
  void getIntersectionsLasersWithModel(const kindr::minimal::QuatTransformation& current_arm_pose,
                                       cad_percept::cgal::Intersection* intersection_a,
                                       cad_percept::cgal::Intersection* intersection_b,
                                       cad_percept::cgal::Intersection* intersection_c);

  /// \brief Performs optimization and returns the pose of the base in the map.
  ///
  /// \param verbose  If True, the optimizer will show verbose prints.
  ///
  /// \return Pose of the base in the map.
  kindr::minimal::QuatTransformation optimizeForBasePoseInMap(bool verbose = false);

 private:
  // Optimizer.
  std::unique_ptr<cad_percept::pointlaser_loc::optimizer::LocalizationOptimizer> optimizer_;
  // Mesh model w.r.t. which localization should be performed.
  cad_percept::cgal::MeshModel::Ptr model_;
  // Noise statistics.
  Eigen::Matrix<double, 6, 1> initial_pose_std_, odometry_noise_std_;
  double pointlaser_noise_std_;
  // Initial pose between arm base and arm link.
  std::unique_ptr<kindr::minimal::QuatTransformation> initial_pose_;
  // Offset between the marker and the arm base.
  std::unique_ptr<kindr::minimal::QuatTransformation> marker_to_armbase_;
  // Offsets between the arm link and the lasers.
  std::unique_ptr<kindr::minimal::QuatTransformation> laser_a_offset_;
  std::unique_ptr<kindr::minimal::QuatTransformation> laser_b_offset_;
  std::unique_ptr<kindr::minimal::QuatTransformation> laser_c_offset_;
  // Goal pose of the arm.
  std::unique_ptr<kindr::minimal::QuatTransformation> arm_goal_pose_;
  // Flags to track the reception of measurements.
  bool was_new_odometry_received_;
  bool were_new_laser_measurements_received_;
};
}  // namespace localizer
}  // namespace pointlaser_loc
}  // namespace cad_percept

#endif  // CPT_POINTLASER_LOC_LOCALIZER_LOCALIZER_H_