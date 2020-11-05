#include "cpt_pointlaser_loc_ros/mabi_localizer.h"

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>
#include <nav_msgs/Path.h>
#include <std_srvs/Empty.h>

#include "cpt_pointlaser_comm_ros/GetDistance.h"

namespace cad_percept {
namespace pointlaser_loc_ros {

MabiLocalizer::MabiLocalizer(ros::NodeHandle &nh, ros::NodeHandle &nh_private,
                             std::string reference_link_topic_name,
                             std::string end_effector_topic_name)
    : nh_(nh),
      nh_private_(nh_private),
      reference_link_topic_name_(reference_link_topic_name),
      end_effector_topic_name_(end_effector_topic_name),
      transform_listener_(nh_),
      task_type_(0) {
  if (!nh_private_.hasParam("off_model")) {
    ROS_ERROR("'off_model' not set as parameter.\n");
  }
  cad_percept::cgal::MeshModel::create(nh_private_.param<std::string>("off_model", "fail"), &model_,
                                       nh_private_.param("verbose", false));

  if (!nh_private_.hasParam("initial_pose_std"))
    ROS_ERROR("'initial_pose_std' not set as parameter.\n");
  initial_armbase_to_ref_link_std_ = Eigen::Matrix<double, 6, 1>(
      nh_private_
          .param<std::vector<double>>("initial_pose_std", std::vector<double>{1, 1, 1, 1, 1, 1})
          .data());
  if (!nh_private_.hasParam("arm_odometry_std"))
    ROS_ERROR("'arm_odometry_std' not set as parameter.\n");
  odometry_noise_std_ = Eigen::Matrix<double, 6, 1>(
      nh_private_
          .param<std::vector<double>>("arm_odometry_std", std::vector<double>{1, 1, 1, 1, 1, 1})
          .data());
  if (!nh_private_.hasParam("pointlaser_noise_std"))
    ROS_ERROR("'pointlaser_noise_std' not set as parameter.\n");
  pointlaser_noise_std_ = nh_private_.param<double>("pointlaser_noise_std", 1.0);

  // Initialize localizer.
  localizer_.reset(new cad_percept::pointlaser_loc::localizer::PointLaserLocalizer(
      model_, initial_armbase_to_ref_link_std_, odometry_noise_std_, pointlaser_noise_std_));

  advertiseTopics();
}

kindr::minimal::QuatTransformation MabiLocalizer::getTF(std::string from, std::string to) {
  tf::StampedTransform transform;
  kindr::minimal::QuatTransformation ret;
  transform_listener_.lookupTransform(from, to, ros::Time(0), transform);
  tf::transformTFToKindr(transform, &ret);
  return ret;
}

void MabiLocalizer::setArmTo(const kindr::minimal::QuatTransformation &arm_goal_pose) {
  ROS_WARN(
      "Please be careful: since no topic/service signaling the end of the arm movement is "
      "available yet, the routine is hard-coded to wait for %f seconds after publishing the "
      "message with the desired goal state.",
      wait_time_arm_movement_);

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
  tf::poseStampedKindrToMsg(arm_goal_pose, ros::Time(motion_duration_), "base", &target_pose_msg);
  path_msg.poses.push_back(target_pose_msg);
  pub_arm_movement_path_.publish(path_msg);
  // TODO(fmilano): Implement an alternative once a proper communication mechanism is available.
  ros::Duration(wait_time_arm_movement_).sleep();
}

bool MabiLocalizer::highAccuracyLocalization(
    cpt_pointlaser_loc_ros::HighAccuracyLocalization::Request &request,
    cpt_pointlaser_loc_ros::HighAccuracyLocalization::Response &response) {
  // Get all the poses.
  // For the links in the arm, check URDF at
  // https://bitbucket.org/leggedrobotics/mabi_common/src/master/mabi_description/.
  // TODO(fmilano): Check that a "marker" is published to the TF!
  kindr::minimal::QuatTransformation marker_to_armbase = getTF("marker", "arm_base");
  kindr::minimal::QuatTransformation initial_pose = getTF("arm_base", reference_link_topic_name_);
  kindr::minimal::QuatTransformation laser_a_offset =
      getTF(reference_link_topic_name_, "pointlaser_A");
  kindr::minimal::QuatTransformation laser_b_offset =
      getTF(reference_link_topic_name_, "pointlaser_B");
  kindr::minimal::QuatTransformation laser_c_offset =
      getTF(reference_link_topic_name_, "pointlaser_C");
  kindr::minimal::QuatTransformation endeffector_offset =
      getTF(reference_link_topic_name_, end_effector_topic_name_);
  kindr::minimal::QuatTransformation arm_base_to_base = getTF("arm_base", "base");

  // Set up the optimizer for a new high-accuracy localization query.
  localizer_->setUpOptimizer(marker_to_armbase, initial_pose, laser_a_offset, laser_b_offset,
                             laser_c_offset, endeffector_offset, arm_base_to_base,
                             nh_private_.param("fix_cad_planes", false),
                             nh_private_.param("initial_pose_prior", false),
                             nh_private_.param("only_optimize_translation", false));
  // We measure over different poses of the end effector and optimize for the pose of the arm base.
  kindr::minimal::QuatTransformation current_pose(initial_pose);

  // Turn laser on, move and measure.
  std_srvs::Empty empty_srvs;
  leica_client_["laserOn"].call(empty_srvs.request, empty_srvs.response);

  if (!nh_private_.hasParam("movement_type_" + std::to_string(task_type_))) {
    ROS_WARN_STREAM("Could not find specific movement type for task-type "
                    << task_type_ << ", using default movement\n");
  }
  // TODO(fmilano): Replace with correct movements!
  std::vector<std::string> default_movement = {
      "0 0 0 0.988 0.009 -0.025 0.05", "0 0 0 0.988 0.009 -0.025 0.05",
      "0 0 0 0.988 0.009 -0.025 0.05", "0 0 0 0.988 0.009 -0.025 0.05",
      "0 0 0 0.988 0.009 -0.025 0.05", "0 0 0 0.988 0.009 -0.025 0.05"};
  // Get the movement from config in string format "x y z w x y z"
  for (auto string_cmd : nh_private_.param<std::vector<std::string>>(
           "movement_type_" + std::to_string(task_type_), default_movement)) {
    // Move arm.
    // Split the string at the spaces
    // (https://www.fluentcpp.com/2017/04/21/how-to-split-a-string-in-c/).
    std::istringstream iss(string_cmd);
    std::vector<std::string> strings((std::istream_iterator<std::string>(iss)),
                                     std::istream_iterator<std::string>());
    if (strings.size() != 7) {
      ROS_ERROR_STREAM("Movement has wrong number of parameters, should be 7 for: " << string_cmd
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
    kindr::minimal::QuatTransformation arm_goal_pose =
        localizer_->getArmGoalPose(rot_quat, translation);
    setArmTo(arm_goal_pose);

    // Add odometry measurement.
    kindr::minimal::QuatTransformation new_arm_pose = getTF("arm_base", reference_link_topic_name_);
    localizer_->addOdometry(current_pose.inverse() * new_arm_pose);
    current_pose = new_arm_pose;

    // Take measurement.
    cpt_pointlaser_comm_ros::GetDistance::Request req;
    cpt_pointlaser_comm_ros::GetDistance::Response resp;
    while (!leica_client_["distance"].call(req, resp)) {
      ROS_ERROR("could not get distance measurement.\n");
      ros::Duration(0.1).sleep();
    }

    localizer_->addLaserMeasurements(resp.distanceA, resp.distanceB, resp.distanceC);

    // For debugging, also publish the intersection as seen from the current state.
    cad_percept::cgal::Intersection model_intersection_a, model_intersection_b,
        model_intersection_c;
    localizer_->getIntersectionsLasersWithModel(current_pose, &model_intersection_a,
                                                &model_intersection_b, &model_intersection_c);
    geometry_msgs::PointStamped intersection_msg;
    intersection_msg.header.stamp = ros::Time::now();
    intersection_msg.header.frame_id = "marker";
    intersection_msg.point.x = model_intersection_a.intersected_point.x();
    intersection_msg.point.y = model_intersection_a.intersected_point.y();
    intersection_msg.point.z = model_intersection_a.intersected_point.z();
    pub_intersection_a_.publish(intersection_msg);
    intersection_msg.point.x = model_intersection_b.intersected_point.x();
    intersection_msg.point.y = model_intersection_b.intersected_point.y();
    intersection_msg.point.z = model_intersection_b.intersected_point.z();
    pub_intersection_b_.publish(intersection_msg);
    intersection_msg.point.x = model_intersection_c.intersected_point.x();
    intersection_msg.point.y = model_intersection_c.intersected_point.y();
    intersection_msg.point.z = model_intersection_c.intersected_point.z();
    pub_intersection_c_.publish(intersection_msg);
  }
  leica_client_["laserOff"].call(empty_srvs.request, empty_srvs.response);

  // Optimize for the pose from the marker to the arm base.
  kindr::minimal::QuatTransformation marker_to_armbase_optimized =
      localizer_->optimizeForArmBasePoseInMap(nh_private_.param<bool>("verbose", false));
  // Translate the pose in the map into a pose in the world frame.
  kindr::minimal::QuatTransformation world_to_armbase =
      getTF("world", "marker") * marker_to_armbase_optimized;

  // Publish pose of the end effector.
  kindr::minimal::QuatTransformation armbase_to_endeffector =
      getTF("arm_base", end_effector_topic_name_);
  geometry_msgs::PoseStamped pose_sent;
  tf::poseKindrToMsg(marker_to_armbase_optimized * armbase_to_endeffector, &pose_sent.pose);
  pose_sent.header.frame_id = "marker";
  pose_sent.header.stamp = ros::Time::now();
  pub_endeffector_pose_.publish(pose_sent);

  // Write pose to terminal.
  ROS_INFO("arm base in marker frame\n");
  ROS_INFO_STREAM(marker_to_armbase_optimized.getPosition().transpose() << "\n");
  ROS_INFO("initial pose from slam\n");
  ROS_INFO_STREAM(marker_to_armbase.getPosition().transpose() << "\n");
  ROS_INFO("arm base in world frame\n");
  ROS_INFO_STREAM(world_to_armbase.getPosition().transpose() << "\n");

  // Send corrected pose of the robot base to the controller.
  kindr::minimal::QuatTransformation base_pose_in_world = world_to_armbase * arm_base_to_base;
  ROS_INFO_STREAM("Updated base pose in world, t: "
                  << base_pose_in_world.getPosition().transpose()
                  << ", o: " << base_pose_in_world.getRotation().vector().transpose() << "\n");
  tf::poseKindrToMsg(base_pose_in_world, &response.corrected_base_pose_in_world);

  return true;
}

void MabiLocalizer::advertiseTopics() {
  // TODO(fmilano): Check these topic names!
  sub_task_type_ = nh_private_.subscribe("/task_type", 10, &MabiLocalizer::setTaskType, this);
  leica_client_["distance"] =
      nh_.serviceClient<cpt_pointlaser_comm_ros::GetDistance>("/pointlaser_comm/distance");
  leica_client_["laserOn"] = nh_.serviceClient<std_srvs::Empty>("/pointlaser_comm/laserOn");
  leica_client_["laserOff"] = nh_.serviceClient<std_srvs::Empty>("/pointlaser_comm/laserOff");

  pub_intersection_a_ = nh_private_.advertise<geometry_msgs::PointStamped>("intersection_a", 1);
  pub_intersection_b_ = nh_private_.advertise<geometry_msgs::PointStamped>("intersection_b", 1);
  pub_intersection_c_ = nh_private_.advertise<geometry_msgs::PointStamped>("intersection_c", 1);
  pub_arm_movement_path_ = nh_private_.advertise<nav_msgs::Path>("hal_base_to_ee_target_pose", 1);
  pub_endeffector_pose_ =
      nh_private_.advertise<geometry_msgs::PoseStamped>("hal_marker_to_end_effector", 1);
  high_acc_localisation_service_ = nh_private_.advertiseService(
      "high_acc_localize", &MabiLocalizer::highAccuracyLocalization, this);
}

void MabiLocalizer::setTaskType(const std_msgs::Int16 &task_type_msg) {
  task_type_ = task_type_msg.data;
}

}  // namespace pointlaser_loc_ros
}  // namespace cad_percept