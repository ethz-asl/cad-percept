#include "cpt_pointlaser_loc_ros/ee_poses_visitor.h"

#include <sstream>
#include <vector>

namespace cad_percept {
namespace pointlaser_loc_ros {

EEPosesVisitor::EEPosesVisitor(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private) {
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

  // Read time (in seconds) - from the sending of the path message to the controller - after which
  // the movement will be assumed to be completed.
  if (!nh_private.hasParam("timeout_arm_movement")) {
    ROS_ERROR("'timeout_arm_movement' not set as parameter.");
  }
  timeout_arm_movement_ = nh_private.param<double>("timeout_arm_movement", 10.0);

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
}

void EEPosesVisitor::readListOfPoses(int movement_type) {
  CHECK(nh_.hasParam("movement_type_" + std::to_string(movement_type)))
      << "Unable to find collection of movements with index " << movement_type
      << " (i.e., the parameters `movement_type_" << movement_type << "` is not defined).";
  kindr::minimal::QuatTransformation base_to_ee_pose;
  std::vector<std::string> movement_collection;
  nh_.getParam("movement_type_" + std::to_string(movement_type), movement_collection);
  for (auto string_cmd : movement_collection) {
    parsePose(string_cmd, &base_to_ee_pose);
    base_to_ee_poses_to_visit_.push_back(base_to_ee_pose);
  }
}

void EEPosesVisitor::setArmTo(const kindr::minimal::QuatTransformation &target_base_to_ee_pose) {}

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
  // TODO(fmilano): fix!
  *parsed_pose = localizer_->getArmGoalPose(rot_quat, translation);
}

void EEPosesVisitor::advertiseAndSubscribe() {
  // Advertise service to wait for start.

  // Subscribe to services of the controller.

  // Subscribe to services of HAL routine.
}

void EEPosesVisitor::goToArmInitialPosition() {
  // Switch controller.
}

void EEPosesVisitor::visitPoses() {
  // Visit pose.

  // Trigger HAL for data collection.

  // When all poses are visited, trigger HAL for optimization.
}

}  // namespace pointlaser_loc_ros
}  // namespace cad_percept