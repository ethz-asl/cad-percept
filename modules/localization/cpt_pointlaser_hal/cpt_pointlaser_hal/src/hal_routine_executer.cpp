#include "cpt_pointlaser_hal/hal_routine_executer.h"

#include <cpt_pointlaser_msgs/EEVisitPose.h>
#include <cpt_pointlaser_msgs/HighAccuracyLocalization.h>
#include <geometry_msgs/Pose.h>
#include <glog/logging.h>
#include <std_srvs/Empty.h>

#include <algorithm>
#include <iostream>
#include <string>

namespace cad_percept {
namespace pointlaser_hal {

HALRoutineExecuter::HALRoutineExecuter(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private) {
  // Subscribe to services.
  hal_move_arm_to_initial_pose_client_ =
      nh_.serviceClient<std_srvs::Empty>("/hal_move_arm_to_initial_pose");
  hal_initialize_localization_client_ =
      nh_.serviceClient<std_srvs::Empty>("/hal_initialize_localization");
  hal_visit_poses_client_ = nh_.serviceClient<cpt_pointlaser_msgs::EEVisitPose>("/hal_visit_poses");
  hal_optimize_client_ =
      nh_.serviceClient<cpt_pointlaser_msgs::HighAccuracyLocalization>("/hal_optimize");
  if (!nh.hasParam("end_effector_topic_name")) {
    ROS_ERROR("'end_effector_topic_name' not set as parameter.");
  }
  optimized_base_frame_name_ = nh.param<std::string>("optimized_base_frame_name", "optimized_base");
  optimized_endeffector_frame_name_ =
      nh.param<std::string>("optimized_endeffector_frame_name", "optimized_ee");

  assistUserThroughRoutine();
}

void HALRoutineExecuter::assistUserThroughRoutine() {
  // Move the arm.
  bool ready = false;
  std::string answer;
  std_srvs::Empty empty_srvs;
  do {
    std::cout << "When ready to move the arm to the initial position, type 'ready' ('q' to exit): ";
    std::cin >> answer;
    std::transform(answer.begin(), answer.end(), answer.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    if (answer == "q") {
      return;
    }
    ready = (answer == "ready");
  } while (!ready);
  CHECK(hal_move_arm_to_initial_pose_client_.exists())
      << "The service to move the arm to its initial pose is not available.";
  CHECK(hal_move_arm_to_initial_pose_client_.call(empty_srvs.request, empty_srvs.response))
      << "Failed to move the arm to its initial pose.";
  // Initialize the HAL localization.
  ready = false;
  do {
    std::cout << "When ready to initialize HAL localization, type 'ready' ('q' to exit): ";
    std::cin >> answer;
    std::transform(answer.begin(), answer.end(), answer.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    if (answer == "q") {
      return;
    }
    ready = (answer == "ready");
  } while (!ready);
  CHECK(hal_initialize_localization_client_.exists())
      << "The service to initialize the localization is not available.";
  CHECK(hal_initialize_localization_client_.call(empty_srvs.request, empty_srvs.response))
      << "Failed to initialize HAL localization.";
  // Visit the poses.
  std::cout << "You may now visit the end-effector poses one at a time and concurrently take "
               "measurements."
            << std::endl;
  bool more_poses_left = true;
  do {
    ready = false;
    do {
      std::cout << "When ready to visit the next end-effector pose, type 'ready' ('q' to exit): ";
      std::cin >> answer;
      std::transform(answer.begin(), answer.end(), answer.begin(),
                     [](unsigned char c) { return std::tolower(c); });
      if (answer == "q") {
        return;
      }
      ready = (answer == "ready");
    } while (!ready);
    cpt_pointlaser_msgs::EEVisitPose hal_visit_pose_srv;
    CHECK(hal_visit_poses_client_.exists()) << "The service to visit poses is not available.";
    CHECK(hal_visit_poses_client_.call(hal_visit_pose_srv.request, hal_visit_pose_srv.response))
        << "Failed to visit end-effector pose.";
    more_poses_left = hal_visit_pose_srv.response.more_poses_left;
  } while (more_poses_left);
  // Optimize for base pose.
  ready = false;
  do {
    std::cout << "All the end-effector poses were visited. When ready to perform HAL optimization, "
                 "type 'ready' ('q' to exit): ";
    std::cin >> answer;
    std::transform(answer.begin(), answer.end(), answer.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    ready = (answer == "ready");
    if (answer == "q") {
      return;
    }
  } while (!ready);
  cpt_pointlaser_msgs::HighAccuracyLocalization hal_optimize_srv;
  CHECK(hal_optimize_client_.exists())
      << "The service to perform HAL optimization is not available.";
  CHECK(hal_optimize_client_.call(hal_optimize_srv.request, hal_optimize_srv.response))
      << "Failed to perform HAL optimization.";
  // Publish optimized pose.
  std::cout << "The HAL routine is now completed. The optimized pose of the base and the "
               "end-effector are published on the TF tree as respectively the new frame '"
            << optimized_base_frame_name_ << "' and the new frame '"
            << optimized_endeffector_frame_name_ << "'." << std::endl;

  geometry_msgs::TransformStamped corrected_map_to_base_msg, corrected_marker_to_endeffector_msg;
  corrected_map_to_base_msg.header.frame_id = "map";
  corrected_map_to_base_msg.child_frame_id = optimized_base_frame_name_;
  corrected_map_to_base_msg.transform = hal_optimize_srv.response.corrected_base_pose_in_map;
  corrected_marker_to_endeffector_msg.header.frame_id = "marker";
  corrected_marker_to_endeffector_msg.child_frame_id = optimized_endeffector_frame_name_;
  corrected_marker_to_endeffector_msg.transform =
      hal_optimize_srv.response.corrected_endeffector_pose_in_marker;
  ros::Rate rate(10.0);
  // Keep publishing the optimized pose until the node is killed.
  do {
    corrected_map_to_base_msg.header.stamp = ros::Time::now();
    corrected_marker_to_endeffector_msg.header.stamp = ros::Time::now();
    tf_broadcaster_.sendTransform(corrected_map_to_base_msg);
    tf_broadcaster_.sendTransform(corrected_marker_to_endeffector_msg);
    rate.sleep();
  } while (nh_.ok());
}

}  // namespace pointlaser_hal
}  // namespace cad_percept
