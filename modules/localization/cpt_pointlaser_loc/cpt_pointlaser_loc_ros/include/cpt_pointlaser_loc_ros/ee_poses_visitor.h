#ifndef CPT_POINTLASER_LOC_ROS_EE_POSES_VISITOR_H_
#define CPT_POINTLASER_LOC_ROS_EE_POSES_VISITOR_H_

#include <kindr/minimal/quat-transformation.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>

namespace cad_percept {
namespace pointlaser_loc_ros {

/// \brief Class that triggers the arm to move through a set of poses to take measurements for the
///   HAL routine.
class EEPosesVisitor {
 public:
  EEPosesVisitor(ros::NodeHandle &nh, ros::NodeHandle &nh_private);

 private:
  ///
  /// \brief Reads from the parameter server the list of poses (expressed in the robot-arm frame)
  ///   through which the end effector should pass when performing the movement.
  ///
  /// \param movement_type Index of the collection of movements to perform, cf.
  ///   `cfg/common_params_mabi.yaml`.
  void readListOfPoses(int movement_type);
  ///
  /// \brief Sets the end effector to a certain pose in the robot-base frame.
  ///
  /// \param target_base_to_ee_pose  Goal pose of the end effector w.r.t. robot base.
  void setArmTo(const kindr::minimal::QuatTransformation &target_base_to_ee_pose);
  ///
  /// \brief Parses a pose from a string in the format "x y z w x y z".
  ///
  /// \param[in] pose_to_parse Pose to parse, in the string format.
  /// \param[out] parsed_pose  Parsed pose.
  /// \return True if the pose could be successfully parsed, false otherwise.
  bool parsePose(const std::string &pose_to_parse, kindr::minimal::QuatTransformation *parsed_pose);

  ///
  /// \brief Converts a relative pose (expressed w.r.t. a reference pose) to an absolute pose.
  ///
  /// \param[in] reference_pose Reference pose w.r.t. which the relative pose is expressed.
  /// \param[in] relative_pose  Relative pose to convert.
  /// \param[out] absolute_pose Absolute pose obtained by converting the input relative pose.
  void relativePoseToAbsolutePose(const kindr::minimal::QuatTransformation &reference_pose,
                                  const kindr::minimal::QuatTransformation &relative_pose,
                                  kindr::minimal::QuatTransformation *absolute_pose);

  void advertiseAndSubscribe();

  bool goToArmInitialPosition(std_srvs::Empty::Request &request,
                              std_srvs::Empty::Response &response);
  bool visitPoses(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);

  // Node handles.
  ros::NodeHandle nh_, nh_private_;
  // Publishers, subscribers.
  // - Publisher of the path that the controller should interpolate to move the arm. The poses are
  //   referred to the end effector and defined in the robot-base frame.
  ros::Publisher arm_movement_path_pub_;
  // Transform listener.
  tf::TransformListener transform_listener_;
  // Service clients and server.
  ros::ServiceClient switch_controller_client_, hal_take_measurement_client_;
  ros::ServiceServer go_to_initial_position_service_, visit_poses_service_;
  // Internal parameters.
  std::string arm_controller_;
  std::string arm_controller_switch_service_name_, path_topic_name_;
  kindr::minimal::QuatTransformation armbase_to_ee_initial_hal_pose_;
  double timeout_arm_movement_, motion_duration_;
  std::string reference_link_topic_name_, end_effector_topic_name_;
  // List of poses that the end-effector should visit, relative to the previous end-effector pose.
  std::vector<kindr::minimal::QuatTransformation> relative_ee_poses_to_visit_;
  // Whether or not the arm was moved to the initial position.
  bool arm_in_initial_position_;
};
}  // namespace pointlaser_loc_ros
}  // namespace cad_percept
#endif
