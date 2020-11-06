#ifndef CPT_POINTLASER_LOC_ROS_MABI_LOCALIZER_H_
#define CPT_POINTLASER_LOC_ROS_MABI_LOCALIZER_H_

#include <cgal_definitions/mesh_model.h>
#include <cpt_pointlaser_loc/localizer/localizer.h>
#include <kindr/minimal/quat-transformation.h>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>

#include <Eigen/Geometry>

#include "cpt_pointlaser_loc_ros/HighAccuracyLocalization.h"

namespace cad_percept {
namespace pointlaser_loc_ros {

/// \brief Class to control and localize the MABI arm.
class MabiLocalizer {
 public:
  ///
  /// \brief Constructs a new MabiLocalizer object.
  ///
  /// \param nh                         Node handle.
  /// \param nh_private                 Private node handle.
  MabiLocalizer(ros::NodeHandle &nh, ros::NodeHandle &nh_private);

 private:
  void advertiseTopics();
  kindr::minimal::QuatTransformation getTF(std::string from, std::string to);
  ///
  /// \brief Initializes the HAL routine, by setting up the optimizer and retrieving the fixed and
  /// initial poses. NOTE: For the way it is currently implemented, this method should not be called
  /// manually, but only by `takeMeasurements` method when the first measurement is taken.
  ///
  /// \return True if the initialization was successful - and in particular if the laser could be
  ///   turned on; false otherwise.
  bool initializeHALRoutine();

  // Service handlers.
  ///
  /// \brief Takes laser measurements from the current pose and adds them, together with
  /// arm-odometry measurements, to the factor graph to be used for the optimization.
  ///
  /// \param request   Service request (empty).
  /// \param response  Service response (empty).
  /// \return True if the measurements could be successfully taken; false otherwise (e.g., if the
  ///   initialization of the HAL routine, required when no previous measurements were taken, is not
  ///   successful.
  bool takeMeasurement(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);
  ///
  /// \brief Performs the actual HAL routine by optimizing over the factor graph previously built.
  ///
  /// \param request   Service request (empty).
  /// \param response  Service response, containing the corrected pose of the robot base as a field.
  /// \return True if the HAL routine is successful, false otherwise (e.g., if the associated
  /// service is called before any measurements are performed).
  bool highAccuracyLocalization(
      cpt_pointlaser_loc_ros::HighAccuracyLocalization::Request &request,
      cpt_pointlaser_loc_ros::HighAccuracyLocalization::Response &response);

  // Reference model.
  cad_percept::cgal::MeshModel::Ptr model_;
  // Node handles.
  ros::NodeHandle nh_, nh_private_;
  // Publisher, subscribers.
  // - Publishers of the intersections of the lasers with the model, for debug purposes.
  ros::Publisher pub_intersection_a_, pub_intersection_b_, pub_intersection_c_;
  // - Publisher of the pose from marker to end-effector.
  ros::Publisher pub_endeffector_pose_;
  // - Subscriber to the initial pose of the arm in the world frame.
  ros::Subscriber sub_offset_pose_;
  // Transform listener.
  tf::TransformListener transform_listener_;
  // Service clients and server.
  std::map<std::string, ros::ServiceClient> leica_client_;
  ros::ServiceServer hal_take_measurement_service_;
  ros::ServiceServer high_acc_localisation_service_;
  // Internal parameters.
  std::string reference_link_topic_name_, end_effector_topic_name_;
  Eigen::Matrix<double, 6, 1> initial_armbase_to_ref_link_std_, odometry_noise_std_;
  double pointlaser_noise_std_;
  bool initialized_hal_routine_;
  kindr::minimal::QuatTransformation initial_marker_to_armbase_;
  kindr::minimal::QuatTransformation current_armbase_to_ref_link_;
  // (Fixed) pose of the robot base w.r.t. to the arm base.
  kindr::minimal::QuatTransformation armbase_to_base_;
  // Localizer that performs the high-accuracy localization task.
  std::unique_ptr<cad_percept::pointlaser_loc::localizer::PointLaserLocalizer> localizer_;
};
}  // namespace pointlaser_loc_ros
}  // namespace cad_percept
#endif
