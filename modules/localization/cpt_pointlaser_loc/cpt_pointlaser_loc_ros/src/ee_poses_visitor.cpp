#include "cpt_pointlaser_loc_ros/ee_poses_visitor.h"

#include <cpt_pointlaser_loc_ros/utils.h>
#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>
#include <nav_msgs/Path.h>
#include <rocoma_msgs/SwitchController.h>

#include <sstream>
#include <vector>

namespace cad_percept {
namespace pointlaser_loc_ros {

EEPosesVisitor::EEPosesVisitor(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private), transform_listener_(nh_), arm_in_initial_position_(false) {
  // Read arm controller to use.
  if (!nh_private.hasParam("arm_controller")) {
    ROS_ERROR("'arm_controller' not set as parameter.");
  }
  arm_controller_ = nh_private.param<std::string>("arm_controller",
                                                  "MabiExternalPathEndeffectorTrackingController");
  // Read name of the service that switches the arm controller.
  if (!nh_private.hasParam("arm_controller_switch_service_name")) {
    ROS_ERROR("'arm_controller_switch_service_name' not set as parameter.");
  }
  arm_controller_switch_service_name_ = nh_private.param<std::string>(
      "arm_controller_switch_service_name",
      "/mabi_mobile_highlevel_controller/controller_manager/switch_controller");

  // Read topic name for the path that the end-effector should follow during the movements.
  if (!nh_private.hasParam("path_topic_name")) {
    ROS_ERROR("'path_topic_name' not set as parameter.");
  }
  path_topic_name_ =
      nh_private.param<std::string>("path_topic_name", "/mission_control/planned_path");

  // Read initial pose in the arm base frame to which the end effector should be set before
  // starting the HAL routine (obtained from URDF).
  if (!nh_private.hasParam("armbase_to_ee_initial_hal_pose")) {
    ROS_ERROR("'armbase_to_ee_initial_hal_pose' not set as parameter.");
  }
  CHECK(parsePose(nh_private.param<std::string>(
                      "armbase_to_ee_initial_hal_pose",
                      "-0.0396287 -0.5516176 0.7162148 -0.7069676 0.7071617 0.0071219 0.0082710"),
                  &armbase_to_ee_initial_hal_pose_))
      << "Unable to parse initial target end-effector pose.";

  // Time (in seconds) - from the sending of the path message to the controller - after which the
  // movement will be assumed to be completed.
  if (!nh_private.hasParam("timeout_arm_movement")) {
    ROS_ERROR("'timeout_arm_movement' not set as parameter.");
  }
  timeout_arm_movement_ = nh_private.param<double>("timeout_arm_movement", 10.0);

  // Fixed duration of the movement in seconds.
  if (!nh_private.hasParam("motion_duration")) {
    ROS_ERROR("'motion_duration' not set as parameter.");
  }
  motion_duration_ = nh_private.param<double>("motion_duration", 3.5);

  // Read list of poses.
  if (!nh_private.hasParam("movement_to_perform")) {
    ROS_ERROR("'movement_to_perform' not set as parameter.");
  }
  int movement_to_perform = nh.param<int>("movement_to_perform", 0);
  readListOfPoses(movement_to_perform);

  // Read topic names.
  if (!nh.hasParam("reference_link_topic_name")) {
    ROS_ERROR("'reference_link_topic_name' not set as parameter.");
  }
  reference_link_topic_name_ = nh.param<std::string>("reference_link_topic_name", "grinder");

  if (!nh.hasParam("end_effector_topic_name")) {
    ROS_ERROR("'end_effector_topic_name' not set as parameter.");
  }
  end_effector_topic_name_ = nh.param<std::string>("end_effector_topic_name", "end_effector");

  advertiseAndSubscribe();
}

void EEPosesVisitor::readListOfPoses(int movement_type) {
  CHECK(nh_.hasParam("movement_type_" + std::to_string(movement_type)))
      << "Unable to find collection of movements with index " << movement_type
      << " (i.e., the parameters `movement_type_" << movement_type << "` is not defined).";
  kindr::minimal::QuatTransformation relative_ee_pose;
  std::vector<std::string> movement_collection;
  nh_.getParam("movement_type_" + std::to_string(movement_type), movement_collection);
  for (auto string_cmd : movement_collection) {
    parsePose(string_cmd, &relative_ee_pose);
    relative_ee_poses_to_visit_.push_back(relative_ee_pose);
  }
}

bool EEPosesVisitor::setArmTo(const kindr::minimal::QuatTransformation &target_base_to_ee_pose) {
  size_t num_subscribers = arm_movement_path_pub_.getNumSubscribers();
  if (num_subscribers != 1) {
    std::string msg;
    if (num_subscribers < 1) {
      msg = "No nodes are";
    } else {
      msg = "More than one node is";
    }
    msg = "Unable to move the arm. " + msg + " subscribed to the path message.\n";
    ROS_ERROR(msg.c_str());
    return false;
  }
  ROS_WARN(
      "Please be careful: since no topic/service signaling the end of the arm movement is "
      "available yet, the routine is hard-coded to wait for %f seconds after publishing the "
      "message with the desired goal state.",
      timeout_arm_movement_);

  nav_msgs::Path path_msg;
  path_msg.header.stamp = ros::Time::now();
  path_msg.header.frame_id = "base";
  // Retrieve current pose from tf and add it to the path.
  // TODO(fmilano): Check that this is the correct way to do this.
  geometry_msgs::PoseStamped current_pose_msg;
  tf::StampedTransform current_pose_tf;
  kindr::minimal::QuatTransformation current_pose;
  transform_listener_.lookupTransform("base", end_effector_topic_name_, ros::Time::now(),
                                      current_pose_tf);
  tf::transformTFToKindr(current_pose_tf, &current_pose);
  tf::poseStampedKindrToMsg(current_pose, ros::Time(0.0), "base", &current_pose_msg);
  path_msg.poses.push_back(current_pose_msg);
  // Add target pose to the path.
  geometry_msgs::PoseStamped target_pose_msg;
  tf::poseStampedKindrToMsg(target_base_to_ee_pose, ros::Time(motion_duration_), "base",
                            &target_pose_msg);
  path_msg.poses.push_back(target_pose_msg);
  arm_movement_path_pub_.publish(path_msg);
  // TODO(fmilano): Implement an alternative once a proper communication mechanism is available.
  ros::Duration(timeout_arm_movement_).sleep();
  ROS_INFO("Assuming movement was completed.\n");

  return true;
}

bool EEPosesVisitor::parsePose(const std::string &pose_to_parse,
                               kindr::minimal::QuatTransformation *parsed_pose) {
  CHECK_NOTNULL(parsed_pose);
  // Split the string at the spaces
  // (https://www.fluentcpp.com/2017/04/21/how-to-split-a-string-in-c/).
  std::istringstream iss(pose_to_parse);
  std::vector<std::string> strings((std::istream_iterator<std::string>(iss)),
                                   std::istream_iterator<std::string>());
  if (strings.size() != 7) {
    ROS_ERROR_STREAM("Movement has wrong number of parameters, should be 7 for: " << pose_to_parse
                                                                                  << "\n");
    return false;
  }
  // Convert strings to doubles.
  std::vector<double> arm_cmd(strings.size());
  std::transform(strings.begin(), strings.end(), arm_cmd.begin(),
                 [](const std::string &s) { return std::stod(s.c_str()); });
  // Compute arm goal pose and move arm to it.
  Eigen::Quaternion<double> rot_quat(arm_cmd[3], arm_cmd[4], arm_cmd[5], arm_cmd[6]);
  kindr::minimal::PositionTemplate<double> translation(arm_cmd[0], arm_cmd[1], arm_cmd[2]);

  kindr::minimal::QuatTransformation parsed_pose_ret(translation, rot_quat);

  *parsed_pose = parsed_pose_ret;

  return true;
}

void EEPosesVisitor::relativePoseToAbsolutePose(
    const kindr::minimal::QuatTransformation &reference_pose,
    const kindr::minimal::QuatTransformation &relative_pose,
    kindr::minimal::QuatTransformation *absolute_pose) {
  Eigen::Quaternion<double> rotation_quat = relative_pose.getEigenQuaternion();
  kindr::minimal::PositionTemplate<double> translation = relative_pose.getPosition();

  kindr::minimal::RotationQuaternionTemplate<double> rotation(rotation_quat);
  kindr::minimal::PositionTemplate<double> no_translation(0, 0, 0);
  kindr::minimal::RotationQuaternionTemplate<double> no_rotation(Eigen::Vector3d(0, 0, 0));

  kindr::minimal::QuatTransformation orientation_shift(no_translation, rotation);
  kindr::minimal::QuatTransformation position_shift(translation, no_rotation);

  *absolute_pose = position_shift * reference_pose * orientation_shift;
}

void EEPosesVisitor::advertiseAndSubscribe() {
  // Advertise services to position the arm and move it through the poses.
  go_to_initial_position_service_ = nh_private_.advertiseService(
      "hal_move_arm_to_initial_pose", &EEPosesVisitor::goToArmInitialPosition, this);
  visit_poses_service_ =
      nh_private_.advertiseService("hal_visit_poses", &EEPosesVisitor::visitPoses, this);
  // Advertise path topic.
  arm_movement_path_pub_ = nh_private_.advertise<nav_msgs::Path>(path_topic_name_, 1);
  // Subscribe to services of the controller.
  switch_controller_client_ =
      nh_.serviceClient<rocoma_msgs::SwitchController>(arm_controller_switch_service_name_);
  // Subscribe to services of HAL routine.
  hal_take_measurement_client_ = nh_.serviceClient<std_srvs::Empty>("hal_take_measurement");
}

bool EEPosesVisitor::goToArmInitialPosition(std_srvs::Empty::Request &request,
                                            std_srvs::Empty::Response &response) {
  // Switch controller.
  rocoma_msgs::SwitchController srv;
  srv.request.name = arm_controller_;
  switch_controller_client_.call(srv);
  if (srv.response.status <= 0) {
    ROS_ERROR("Failed to switch arm controller on.");
    return false;
  }
  // Move arm to initial position.
  kindr::minimal::QuatTransformation base_to_armbase_pose = getTF("base", "arm_base");
  kindr::minimal::QuatTransformation base_to_ee_initial_hal_pose =
      base_to_armbase_pose * armbase_to_ee_initial_hal_pose_;
  if (setArmTo(base_to_ee_initial_hal_pose)) {
    arm_in_initial_position_ = true;
    return true;
  } else {
    return false;
  }
}

bool EEPosesVisitor::visitPoses(std_srvs::Empty::Request &request,
                                std_srvs::Empty::Response &response) {
  if (!arm_in_initial_position_) {
    ROS_ERROR(
        "The arm was not moved to its initial pose! Please call the service "
        "`hal_move_arm_to_initial_pose` first.");
    return false;
  }
  kindr::minimal::QuatTransformation current_base_to_ee_pose = armbase_to_ee_initial_hal_pose_;
  arm_in_initial_position_ = false;
  for (const auto &relative_pose : relative_ee_poses_to_visit_) {
    // Visit pose.
    relativePoseToAbsolutePose(current_base_to_ee_pose, relative_pose, &current_base_to_ee_pose);
    CHECK(setArmTo(current_base_to_ee_pose)) << "Unable to perform arm movement.";
    // Trigger HAL for data collection.
    std_srvs::Empty srv_data_collection;
    CHECK(hal_take_measurement_client_.call(srv_data_collection))
        << "Failure to collect data after movement was performed.";
  }

  ROS_INFO(
      "All poses were visited and all measurement were collected. You may now call the service to "
      "perform the HAL optimization.");

  return true;
}

}  // namespace pointlaser_loc_ros
}  // namespace cad_percept