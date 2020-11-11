#include "cpt_pointlaser_loc_ros/mabi_localizer.h"

#include <cgal_conversions/mesh_conversions.h>
#include <cpt_pointlaser_comm_ros/GetDistance.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>
#include <nav_msgs/Path.h>
#include <std_srvs/Empty.h>

namespace cad_percept {
namespace pointlaser_loc_ros {

MabiLocalizer::MabiLocalizer(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      transform_listener_(nh_),
      initialized_hal_routine_(false),
      received_at_least_one_measurement_(false),
      received_cad_model_(false) {
  if (!nh_private_.hasParam("initial_pose_std")) {
    ROS_ERROR("'initial_pose_std' not set as parameter.");
  }
  initial_armbase_to_ref_link_std_ = Eigen::Matrix<double, 6, 1>(
      nh_private_
          .param<std::vector<double>>("initial_pose_std", std::vector<double>{1, 1, 1, 1, 1, 1})
          .data());
  if (!nh_private_.hasParam("arm_odometry_std")) {
    ROS_ERROR("'arm_odometry_std' not set as parameter.");
  }
  odometry_noise_std_ = Eigen::Matrix<double, 6, 1>(
      nh_private_
          .param<std::vector<double>>("arm_odometry_std", std::vector<double>{1, 1, 1, 1, 1, 1})
          .data());
  if (!nh_private_.hasParam("pointlaser_noise_std")) {
    ROS_ERROR("'pointlaser_noise_std' not set as parameter.");
  }
  pointlaser_noise_std_ = nh_private_.param<double>("pointlaser_noise_std", 1.0);

  if (!nh_private.hasParam("reference_link_topic_name")) {
    ROS_ERROR("'reference_link_topic_name' not set as parameter.");
  }
  reference_link_topic_name_ =
      nh_private_.param<std::string>("reference_link_topic_name", "grinder");

  if (!nh_private.hasParam("end_effector_topic_name")) {
    ROS_ERROR("'end_effector_topic_name' not set as parameter.");
  }
  end_effector_topic_name_ =
      nh_private_.param<std::string>("end_effector_topic_name", "end_effector");

  advertiseTopics();
}

kindr::minimal::QuatTransformation MabiLocalizer::getTF(std::string from, std::string to) {
  tf::StampedTransform transform;
  kindr::minimal::QuatTransformation ret;
  transform_listener_.lookupTransform(from, to, ros::Time(0), transform);
  tf::transformTFToKindr(transform, &ret);
  return ret;
}

void MabiLocalizer::modelCallback(const cgal_msgs::TriangleMeshStamped &cad_mesh_msg) {
  if (received_cad_model_) {
    // Model was already received.
    return;
  }
  // Load CAD model.
  ROS_INFO("Loading CAD model.");
  cgal::msgToMeshModel(cad_mesh_msg.mesh, &model_);

  received_cad_model_ = true;

  // Initialize localizer.
  localizer_.reset(new cad_percept::pointlaser_loc::localizer::PointLaserLocalizer(
      model_, initial_armbase_to_ref_link_std_, odometry_noise_std_, pointlaser_noise_std_));
}

bool MabiLocalizer::initializeHALRoutine() {
  if (!received_cad_model_) {
    ROS_ERROR("CAD model was not received. Unable to initialize localizer and run HAL routine.");
    return false;
  }
  // NOTE: It is assumed that the arm was already moved to its initial pose.
  ROS_INFO(
      "Initializing HAL routine. NOTE: It is assumed that the arm was already moved to its initial "
      "pose.");
  // Get all the poses.
  // For the links in the arm, check URDF at
  // https://bitbucket.org/leggedrobotics/mabi_common/src/master/mabi_description/.
  initial_marker_to_armbase_ = getTF("marker", "arm_base");
  kindr::minimal::QuatTransformation initial_pose = getTF("arm_base", reference_link_topic_name_);
  kindr::minimal::QuatTransformation laser_a_offset =
      getTF(reference_link_topic_name_, "pointlaser_A");
  kindr::minimal::QuatTransformation laser_b_offset =
      getTF(reference_link_topic_name_, "pointlaser_B");
  kindr::minimal::QuatTransformation laser_c_offset =
      getTF(reference_link_topic_name_, "pointlaser_C");
  kindr::minimal::QuatTransformation endeffector_offset =
      getTF(reference_link_topic_name_, end_effector_topic_name_);
  armbase_to_base_ = getTF("arm_base", "base");
  // Set up the optimizer for a new high-accuracy localization query.
  localizer_->setUpOptimizer(initial_marker_to_armbase_, initial_pose, laser_a_offset,
                             laser_b_offset, laser_c_offset, endeffector_offset, armbase_to_base_,
                             nh_private_.param("fix_cad_planes", false),
                             nh_private_.param("initial_pose_prior", false),
                             nh_private_.param("only_optimize_translation", false));
  // We measure over different poses of the end effector and optimize for the pose of the reference
  // link w.r.t. the arm base. Eventually, this is used to retrieve the corrected pose of the robot
  // base in the world frame (cf. `highAccuracyLocalization`).
  current_armbase_to_ref_link_ = initial_pose;

  // Turn laser on.
  std_srvs::Empty empty_srvs;
  if (!leica_client_["laserOn"].call(empty_srvs.request, empty_srvs.response)) {
    ROS_ERROR("Failed to turn laser on. Unable to initialize HAL routine.");
    return false;
  }
  initialized_hal_routine_ = true;
  return true;
}

bool MabiLocalizer::takeMeasurement(std_srvs::Empty::Request &request,
                                    std_srvs::Empty::Response &response) {
  // Initialize HAL routine if not previously done.
  if (!initialized_hal_routine_) {
    if (!initializeHALRoutine()) {
      ROS_ERROR("Unable to take measurement because the HAL routine could not be initialized.");
      return false;
    }
  }
  // Take measurements.
  ROS_INFO("Taking laser measurements.");
  // - Add odometry measurement to the factor graph.
  kindr::minimal::QuatTransformation new_arm_pose = getTF("arm_base", reference_link_topic_name_);
  localizer_->addOdometry(current_armbase_to_ref_link_.inverse() * new_arm_pose);
  current_armbase_to_ref_link_ = new_arm_pose;
  // - Take laser measurements and add them to the factor graph.
  cpt_pointlaser_comm_ros::GetDistance::Request req;
  cpt_pointlaser_comm_ros::GetDistance::Response resp;
  while (!leica_client_["distance"].call(req, resp)) {
    ROS_ERROR("Could not get distance measurement.");
    ros::Duration(0.1).sleep();
  }
  localizer_->addLaserMeasurements(resp.distanceA, resp.distanceB, resp.distanceC);

  // For debugging, also publish the intersection as seen from the current state.
  cad_percept::cgal::Intersection model_intersection_a, model_intersection_b, model_intersection_c;
  localizer_->getIntersectionsLasersWithModel(current_armbase_to_ref_link_, &model_intersection_a,
                                              &model_intersection_b, &model_intersection_c);
  geometry_msgs::PointStamped intersection_msg;
  intersection_msg.header.stamp = ros::Time::now();
  intersection_msg.header.frame_id = "marker";
  intersection_msg.point.x = model_intersection_a.intersected_point.x();
  intersection_msg.point.y = model_intersection_a.intersected_point.y();
  intersection_msg.point.z = model_intersection_a.intersected_point.z();
  intersection_a_pub_.publish(intersection_msg);
  intersection_msg.point.x = model_intersection_b.intersected_point.x();
  intersection_msg.point.y = model_intersection_b.intersected_point.y();
  intersection_msg.point.z = model_intersection_b.intersected_point.z();
  intersection_b_pub_.publish(intersection_msg);
  intersection_msg.point.x = model_intersection_c.intersected_point.x();
  intersection_msg.point.y = model_intersection_c.intersected_point.y();
  intersection_msg.point.z = model_intersection_c.intersected_point.z();
  intersection_c_pub_.publish(intersection_msg);

  received_at_least_one_measurement_ = true;

  return true;
}

bool MabiLocalizer::highAccuracyLocalization(
    cpt_pointlaser_msgs::HighAccuracyLocalization::Request &request,
    cpt_pointlaser_msgs::HighAccuracyLocalization::Response &response) {
  if (!received_at_least_one_measurement_) {
    ROS_ERROR(
        "Unable to perform the HAL routine. No measurements were received since the last completed "
        "HAL routine or the HAL routine was not (re-)initialized.");
    return false;
  }
  // Turn the laser off.
  std_srvs::Empty empty_srvs;
  leica_client_["laserOff"].call(empty_srvs.request, empty_srvs.response);

  // Optimize for the pose from the marker to the arm base.
  kindr::minimal::QuatTransformation marker_to_armbase_optimized =
      localizer_->optimizeForArmBasePoseInMap(nh_private_.param<bool>("verbose_optimizer", false));
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
  endeffector_pose_pub_.publish(pose_sent);

  // Write pose to terminal.
  ROS_INFO("Arm base pose in marker frame:");
  ROS_INFO_STREAM(marker_to_armbase_optimized.getPosition().transpose() << "\n");
  ROS_INFO("Initial arm base pose in marker frame from state estimation:");
  ROS_INFO_STREAM(initial_marker_to_armbase_.getPosition().transpose() << "\n");
  ROS_INFO("Arm base pose in world frame:");
  ROS_INFO_STREAM(world_to_armbase.getPosition().transpose() << "\n");

  // Send corrected pose of the robot base to the controller.
  kindr::minimal::QuatTransformation base_pose_in_world = world_to_armbase * armbase_to_base_;
  ROS_INFO_STREAM("Updated base pose in world, t: "
                  << base_pose_in_world.getPosition().transpose()
                  << ", o: " << base_pose_in_world.getRotation().vector().transpose() << "\n");
  tf::poseKindrToMsg(base_pose_in_world, &response.corrected_base_pose_in_world);

  // Set the HAL routine as completed.
  initialized_hal_routine_ = false;
  received_at_least_one_measurement_ = false;

  return true;
}

void MabiLocalizer::advertiseTopics() {
  cad_model_sub_ =
      nh_.subscribe("/mesh_publisher/mesh_out", 10, &MabiLocalizer::modelCallback, this);

  leica_client_["distance"] =
      nh_.serviceClient<cpt_pointlaser_comm_ros::GetDistance>("/pointlaser_comm/distance");
  leica_client_["laserOn"] = nh_.serviceClient<std_srvs::Empty>("/pointlaser_comm/laserOn");
  leica_client_["laserOff"] = nh_.serviceClient<std_srvs::Empty>("/pointlaser_comm/laserOff");

  intersection_a_pub_ = nh_private_.advertise<geometry_msgs::PointStamped>("intersection_a", 1);
  intersection_b_pub_ = nh_private_.advertise<geometry_msgs::PointStamped>("intersection_b", 1);
  intersection_c_pub_ = nh_private_.advertise<geometry_msgs::PointStamped>("intersection_c", 1);
  endeffector_pose_pub_ =
      nh_private_.advertise<geometry_msgs::PoseStamped>("hal_marker_to_end_effector", 1);
  high_acc_localisation_service_ = nh_private_.advertiseService(
      "high_accuracy_localize", &MabiLocalizer::highAccuracyLocalization, this);
  hal_take_measurement_service_ =
      nh_private_.advertiseService("hal_take_measurement", &MabiLocalizer::takeMeasurement, this);
}

}  // namespace pointlaser_loc_ros
}  // namespace cad_percept