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
  /// \param model                            The mesh model w.r.t. which localization should be
  ///   performed.
  /// \param initial_armbase_to_ref_link_std  Standard deviation of the initial pose from arm base
  ///   to reference link.
  /// \param odometry_noise_std               Standard deviation of the odometry.
  /// TODO(fmilano): Check, the `odometry_noise_std` is not used internally by optimizer.
  /// \param pointlaser_noise_std             Standard deviation of the point-laser noise.
  PointLaserLocalizer(const cad_percept::cgal::MeshModel::Ptr& model,
                      const Eigen::Matrix<double, 6, 1>& initial_armbase_to_ref_link_std,
                      const Eigen::Matrix<double, 6, 1>& odometry_noise_std,
                      double pointlaser_noise_std);

  /// \brief Sets up the optimizer for a new high-accuracy localization query.
  /// NOTE: it is assumed that the arm was already moved to the initial pose. A reference link in
  /// the arm, with a fixed pose w.r.t. to the lasers, is used: its initial pose w.r.t. to the arm
  /// base must be provided as input to the optimizer, which optimizes for the pose from the marker
  /// to the arm base (cf. `optimizeForArmBasePoseInMap`).
  ///
  /// \param marker_to_armbase            (Fixed) pose from the marker to the arm base.
  /// \param initial_armbase_to_ref_link  Initial pose from the arm base to the reference
  ///   link.
  /// \param laser_a_offset               (Fixed) pose from the reference link to the point laser A.
  /// \param laser_b_offset               (Fixed) pose from the reference link to the point laser B.
  /// \param laser_c_offset               (Fixed) pose from the reference link to the point laser C.
  /// \param fix_cad_planes               If True, the planes retrieved by the optimizer are fixed.
  /// \param add_marker_pose_prior        If True, a factor for the prior of the model offset (from
  ///   arm base to marker) is added to the optimization graph.
  /// \param only_optimize_translation    If True, only optimizes translation.
  /// \return True if localization can be successfully performed, False otherwise.
  bool setUpOptimizer(const kindr::minimal::QuatTransformation& marker_to_armbase,
                      const kindr::minimal::QuatTransformation& initial_armbase_to_ref_link,
                      const kindr::minimal::QuatTransformation& laser_a_offset,
                      const kindr::minimal::QuatTransformation& laser_b_offset,
                      const kindr::minimal::QuatTransformation& laser_c_offset,
                      bool fix_cad_planes = false, bool add_marker_pose_prior = false,
                      bool only_optimize_translation = false);

  /// \brief Adds an odometry transform (on the reference link) to the optimization.
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
  /// \param[in] current_arm_pose  Current pose of the reference link w.r.t. the arm base.
  /// \param[out] intersection_a   Intersection of laser A with the model.
  /// \param[out] intersection_b   Intersection of laser B with the model.
  /// \param[out] intersection_c   Intersection of laser C with the model.
  ///
  /// \return None.
  void getIntersectionsLasersWithModel(const kindr::minimal::QuatTransformation& current_arm_pose,
                                       cad_percept::cgal::Intersection* intersection_a,
                                       cad_percept::cgal::Intersection* intersection_b,
                                       cad_percept::cgal::Intersection* intersection_c);

  /// \brief Performs optimization and returns the pose of the arm base in the map, i.e., the pose
  /// from marker to arm base.
  ///
  /// \param verbose  If True, the optimizer will show verbose prints.
  ///
  /// \return Pose of the base in the map.
  kindr::minimal::QuatTransformation optimizeForArmBasePoseInMap(bool verbose = false);

 private:
  // Optimizer.
  std::unique_ptr<cad_percept::pointlaser_loc::optimizer::LocalizationOptimizer> optimizer_;
  // Mesh model w.r.t. which localization should be performed.
  cad_percept::cgal::MeshModel::Ptr model_;
  // Noise statistics.
  Eigen::Matrix<double, 6, 1> initial_armbase_to_ref_link_std_, odometry_noise_std_;
  double pointlaser_noise_std_;
  // Initial pose from arm base to reference link.
  std::unique_ptr<kindr::minimal::QuatTransformation> initial_armbase_to_ref_link_;
  // Offset from the marker to the arm base.
  std::unique_ptr<kindr::minimal::QuatTransformation> marker_to_armbase_;
  // Offsets from the reference link to the lasers.
  std::unique_ptr<kindr::minimal::QuatTransformation> laser_a_offset_;
  std::unique_ptr<kindr::minimal::QuatTransformation> laser_b_offset_;
  std::unique_ptr<kindr::minimal::QuatTransformation> laser_c_offset_;
  // Flags to track the reception of measurements.
  bool was_new_odometry_received_;
  bool were_new_laser_measurements_received_;
};
}  // namespace localizer
}  // namespace pointlaser_loc
}  // namespace cad_percept

#endif  // CPT_POINTLASER_LOC_LOCALIZER_LOCALIZER_H_