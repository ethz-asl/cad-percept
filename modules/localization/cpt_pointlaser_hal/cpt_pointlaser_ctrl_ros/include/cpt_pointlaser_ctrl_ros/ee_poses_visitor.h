#ifndef CPT_POINTLASER_CTRL_ROS_EE_POSES_VISITOR_H_
#define CPT_POINTLASER_CTRL_ROS_EE_POSES_VISITOR_H_

#include <cpt_pointlaser_msgs/AlignPointlaserAToMarker.h>
#include <cpt_pointlaser_msgs/EEVisitPose.h>
#include <cpt_pointlaser_msgs/RotatePointlaserA.h>
#include <kindr/minimal/quat-transformation.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>

namespace cad_percept {
namespace pointlaser_ctrl_ros {

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
  /// \return True if (exactly) one node is listening to the path message containing the poses - and
  ///   therefore it is assumed that the movement could be performed -, false otherwise.
  bool setArmTo(const kindr::minimal::QuatTransformation &target_base_to_ee_pose);
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
  ///
  /// \brief Aligns pointlaser A to the x axis of the marker frame.
  bool alignPointlaserAToMarker(cpt_pointlaser_msgs::AlignPointlaserAToMarker::Request &request,
                                cpt_pointlaser_msgs::AlignPointlaserAToMarker::Response &response);
  ///
  /// \brief Rotates pointlaser A according to the input rotation. NOTE: The reference pose w.r.t.
  ///   the rotation is defined is NOT (necessarily) the current pose of the pointlaser, but rather
  ///   the initial pose to which the pointlaser is moved when aligning it to the marker (cf.
  ///   `alignPointlaserAToMarker`).
  bool rotatePointlaserA(cpt_pointlaser_msgs::RotatePointlaserA::Request &request,
                         cpt_pointlaser_msgs::RotatePointlaserA::Response &response);
  bool visitPoses(cpt_pointlaser_msgs::EEVisitPose::Request &request,
                  cpt_pointlaser_msgs::EEVisitPose::Response &response);

  // Node handles.
  ros::NodeHandle nh_, nh_private_;
  // Publishers, subscribers.
  // - Publisher of the path that the controller should interpolate to move the arm. The poses are
  //   referred to the end effector and defined in the robot-base frame.
  ros::Publisher arm_movement_path_pub_;
  // Transform listener.
  tf::TransformListener transform_listener_;
  // Service clients and server.
  ros::ServiceClient switch_arm_controller_client_, switch_combined_controller_client_,
      hal_take_measurement_client_;
  ros::ServiceServer go_to_initial_position_service_, visit_poses_service_,
      align_pointlaser_A_to_marker_service_, rotate_pointlaser_A_service_;
  // Internal parameters.
  std::string arm_controller_, combined_controller_;
  std::string arm_controller_switch_service_name_, combined_controller_switch_service_name_,
      path_topic_name_;
  kindr::minimal::QuatTransformation armbase_to_ee_initial_hal_pose_;
  double timeout_arm_movement_, motion_duration_;
  std::string reference_link_topic_name_, end_effector_topic_name_;
  // List of poses that the end-effector should visit, relative to the previous end-effector pose.
  std::vector<kindr::minimal::QuatTransformation> relative_ee_poses_to_visit_;
  // Whether or not the arm was moved to the initial position.
  bool arm_in_initial_position_;
  // Whether or not the lasers were aligned to the marker frame.
  bool lasers_aligned_with_marker_;
  // Current pose of the end-effector in the robot-base frame.
  kindr::minimal::QuatTransformation current_base_to_ee_pose_;
  // Initial pose of the pointlaser A w.r.t. the marker frame, after aligning the laser with the
  // marker frame. Used in RL tests as a reference w.r.t. compute the subsequent poses.
  kindr::minimal::QuatTransformation initial_marker_to_pointlaser_A_pose_;
  // Number of poses visited.
  size_t num_poses_visited_;
  // Whether or not the node is running in simulation node (needed to select the controllers).
  bool simulation_mode_;
};
}  // namespace pointlaser_ctrl_ros
}  // namespace cad_percept
#endif
