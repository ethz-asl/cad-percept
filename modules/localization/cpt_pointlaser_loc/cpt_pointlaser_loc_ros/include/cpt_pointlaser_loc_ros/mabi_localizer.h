#ifndef CPT_POINTLASER_LOC_ROS_MABI_LOCALIZER_H_
#define CPT_POINTLASER_LOC_ROS_MABI_LOCALIZER_H_

#include <cgal_definitions/mesh_model.h>
#include <cgal_msgs/TriangleMeshStamped.h>
#include <cpt_pointlaser_loc/localizer/localizer.h>
#include <cpt_pointlaser_msgs/HighAccuracyLocalization.h>
#include <kindr/minimal/quat-transformation.h>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_srvs/Empty.h>

#include <Eigen/Geometry>

namespace cad_percept {
namespace pointlaser_loc_ros {

/// \brief Class to perform high-accuracy localization of the robot with the MABI arm.
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

  // Callbacks.
  ///
  /// \brief Callback for the topic that publishes the CAD model.
  ///
  /// \param cad_mesh_msg Message containing the building model message.
  void modelCallback(const cgal_msgs::TriangleMeshStamped &cad_mesh_msg);

  // Service handlers.
  ///
  /// \brief Initializes the HAL routine, by setting up the optimizer and retrieving the fixed and
  ///   initial poses. NOTE: It assumes that the arm was already moved to its initial position.
  ///
  /// \param request   Service request (empty).
  /// \param response  Service response (empty).
  /// \return True if the initialization was successful - and in particular if the laser could be
  ///   turned on; false otherwise.
  bool initializeHALLocalization(std_srvs::Empty::Request &request,
                                 std_srvs::Empty::Response &response);

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
  ///   service is called before any measurements are performed).
  bool highAccuracyLocalization(cpt_pointlaser_msgs::HighAccuracyLocalization::Request &request,
                                cpt_pointlaser_msgs::HighAccuracyLocalization::Response &response);

  // Reference model.
  cad_percept::cgal::MeshModel::Ptr model_;
  // Node handles.
  ros::NodeHandle nh_, nh_private_;
  // Publisher, subscribers.
  // - Publishers of the intersections of the lasers with the model, for debug purposes.
  ros::Publisher intersection_a_pub_, intersection_b_pub_, intersection_c_pub_;
  // - Publisher of the pose from marker to end-effector.
  ros::Publisher endeffector_pose_pub_;
  // - Subscriber to the CAD model.
  ros::Subscriber cad_model_sub_;
  // Service clients and server.
  std::map<std::string, ros::ServiceClient> leica_client_;
  ros::ServiceServer hal_initialize_localization_service_;
  ros::ServiceServer hal_take_measurement_service_;
  ros::ServiceServer hal_optimize_service_;
  // Internal parameters.
  std::string reference_link_topic_name_, end_effector_topic_name_;
  Eigen::Matrix<double, 6, 1> initial_armbase_to_ref_link_std_, odometry_noise_std_;
  double pointlaser_noise_std_;
  bool received_cad_model_;
  bool received_at_least_one_measurement_;
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
