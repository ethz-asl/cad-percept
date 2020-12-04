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
  hal_visit_poses_client_ = nh_.serviceClient<std_srvs::Empty>("/hal_visit_poses");
  hal_optimize_client_ =
      nh_.serviceClient<cpt_pointlaser_msgs::HighAccuracyLocalization>("/hal_optimize");
  // Advertise topic with corrected base pose.
  corrected_base_pose_in_world_pub_ =
      nh_private_.advertise<geometry_msgs::Pose>(optimized_base_pose_topic_name_, 1);

  if (!nh_.hasParam("optimized_base_pose_topic_name")) {
    ROS_ERROR("'optimized_base_pose_topic_name' not set as parameter.");
  }
  optimized_base_pose_topic_name_ =
      nh_.param<std::string>("optimized_base_pose_topic_name", "hal_corrected_base_pose_in_world");

  assistUserThroughRoutine();
}

void HALRoutineExecuter::assistUserThroughRoutine() {
  // Move the arm.
  bool ready = false;
  std::string answer;
  std_srvs::Empty empty_srvs;
  do {
    std::cout << "When ready to move the arm to the initial position, type 'ready': ";
    std::cin >> answer;
    std::transform(answer.begin(), answer.end(), answer.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    ready = (answer == "ready");
  } while (!ready);
  CHECK(hal_move_arm_to_initial_pose_client_.call(empty_srvs.request, empty_srvs.response))
      << "Failed to move the arm to its initial pose.";
  // Initialize the HAL localization.
  ready = false;
  do {
    std::cout << "When ready to initialize HAL localization, type 'ready': ";
    std::cin >> answer;
    std::transform(answer.begin(), answer.end(), answer.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    ready = (answer == "ready");
  } while (!ready);
  CHECK(hal_initialize_localization_client_.call(empty_srvs.request, empty_srvs.response))
      << "Failed to initialize HAL localization.";
  // Visit the poses.
  std::cout << "You may now visit the end-effector poses one at a time and concurrently take "
               "measurements.";
  bool more_poses_left = true;
  do {
    ready = false;
    do {
      std::cout << "When ready to visit the next end-effector pose, type 'ready': ";
      std::cin >> answer;
      std::transform(answer.begin(), answer.end(), answer.begin(),
                     [](unsigned char c) { return std::tolower(c); });
      ready = (answer == "ready");
    } while (!ready);
    cpt_pointlaser_msgs::EEVisitPose hal_visit_pose_srv;
    CHECK(hal_visit_poses_client_.call(hal_visit_pose_srv.request, hal_visit_pose_srv.response))
        << "Failed to visit end-effector pose.";
    more_poses_left = hal_visit_pose_srv.response.more_poses_left;
  } while (more_poses_left);
  // Optimize for base pose.
  ready = false;
  do {
    std::cout << "All the end-effector poses were visited. When ready to perform HAL optimization, "
                 "type 'ready': ";
    std::cin >> answer;
    std::transform(answer.begin(), answer.end(), answer.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    ready = (answer == "ready");
  } while (!ready);
  cpt_pointlaser_msgs::HighAccuracyLocalization hal_optimize_srv;
  CHECK(hal_optimize_client_.call(hal_optimize_srv.request, hal_optimize_srv.response))
      << "Failed to perform HAL optimization.";
  // Publish optimized pose.
  corrected_base_pose_in_world_pub_.publish(hal_optimize_srv.response);

  std::cout << "The HAL routine is now completed. The optimized pose of the base in the world "
               "frame is published on the topic '"
            << optimized_base_pose_topic_name_ << "'." << std::endl;
}

}  // namespace pointlaser_hal
}  // namespace cad_percept
