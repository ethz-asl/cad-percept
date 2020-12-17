#include "cpt_pointlaser_loc_ros/mabi_localizer.h"

#include <cgal_conversions/mesh_conversions.h>
#include <cgal_msgs/ColoredMesh.h>
#include <cgal_msgs/TriangleMesh.h>
#include <cpt_pointlaser_comm_ros/GetDistance.h>
#include <cpt_pointlaser_common/utils.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <minkindr_conversions/kindr_msg.h>
#include <nav_msgs/Path.h>
#include <std_msgs/ColorRGBA.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

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

  if (!nh.hasParam("reference_link_topic_name")) {
    ROS_ERROR("'reference_link_topic_name' not set as parameter.");
  }
  reference_link_topic_name_ = nh.param<std::string>("reference_link_topic_name", "grinder");

  if (!nh.hasParam("end_effector_topic_name")) {
    ROS_ERROR("'end_effector_topic_name' not set as parameter.");
  }
  end_effector_topic_name_ = nh.param<std::string>("end_effector_topic_name", "end_effector");

  advertiseTopics();
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

bool MabiLocalizer::initializeHALLocalization(std_srvs::Empty::Request &request,
                                              std_srvs::Empty::Response &response) {
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
  initial_marker_to_armbase_ =
      cad_percept::pointlaser_common::getTF(transform_listener_, "marker", "arm_base");
  kindr::minimal::QuatTransformation initial_pose = cad_percept::pointlaser_common::getTF(
      transform_listener_, "arm_base", reference_link_topic_name_);
  kindr::minimal::QuatTransformation laser_a_offset = cad_percept::pointlaser_common::getTF(
      transform_listener_, reference_link_topic_name_, "pointlaser_A");
  kindr::minimal::QuatTransformation laser_b_offset = cad_percept::pointlaser_common::getTF(
      transform_listener_, reference_link_topic_name_, "pointlaser_B");
  kindr::minimal::QuatTransformation laser_c_offset = cad_percept::pointlaser_common::getTF(
      transform_listener_, reference_link_topic_name_, "pointlaser_C");
  armbase_to_base_ = cad_percept::pointlaser_common::getTF(transform_listener_, "arm_base", "base");
  // Set up the optimizer for a new high-accuracy localization query.
  localizer_->setUpOptimizer(
      initial_marker_to_armbase_, initial_pose, laser_a_offset, laser_b_offset, laser_c_offset,
      nh_private_.param("fix_cad_planes", false), nh_private_.param("initial_pose_prior", false),
      nh_private_.param("only_optimize_translation", false));
  // We measure over different poses of the end effector and optimize for the pose of the reference
  // link w.r.t. the arm base. Eventually, this is used to retrieve the corrected pose of the robot
  // base in the map frame (cf. `highAccuracyLocalization`).
  current_armbase_to_ref_link_ = initial_pose;

  // Turn laser on.
  std_srvs::Empty empty_srvs;
  if (!leica_client_["laserOn"].exists()) {
    ROS_ERROR("The service to turn the laser on is not available.");
    return false;
  }
  if (!leica_client_["laserOn"].call(empty_srvs.request, empty_srvs.response)) {
    ROS_ERROR("Failed to turn laser on. Unable to initialize HAL routine.");
    return false;
  }
  initialized_hal_routine_ = true;
  return true;
}

bool MabiLocalizer::takeMeasurement(std_srvs::Empty::Request &request,
                                    std_srvs::Empty::Response &response) {
  if (!initialized_hal_routine_) {
    ROS_ERROR("Unable to take measurement because the HAL routine was not initialized.");
    return false;
  }
  // Take measurements.
  ROS_INFO("Taking laser measurements.");
  // - Add odometry measurement to the factor graph.
  kindr::minimal::QuatTransformation new_arm_pose = cad_percept::pointlaser_common::getTF(
      transform_listener_, "arm_base", reference_link_topic_name_);
  localizer_->addOdometry(current_armbase_to_ref_link_.inverse() * new_arm_pose);
  current_armbase_to_ref_link_ = new_arm_pose;
  // - Take laser measurements and add them to the factor graph.
  cpt_pointlaser_comm_ros::GetDistance::Request req;
  cpt_pointlaser_comm_ros::GetDistance::Response resp;
  while (!leica_client_["distance"].call(req, resp)) {
    ROS_ERROR("Could not get distance measurement.");
    ros::Duration(0.1).sleep();
  }

  std::vector<Eigen::Vector3d> intersected_plane_normals, intersected_plane_supports;
  std::vector<std::string> intersected_face_ids;
  localizer_->addLaserMeasurements(resp.distanceA, resp.distanceB, resp.distanceC,
                                   &intersected_plane_normals, &intersected_plane_supports,
                                   &intersected_face_ids);

  // For debugging, also publish the intersection as seen from the current state.
  // - Intersections using ray from current pose.
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
  // - Intersections obtained from querying building model.
  visualization_msgs::Marker marker;
  visualization_msgs::MarkerArray marker_array;
  geometry_msgs::Point p_from, p_to;
  marker.header.frame_id = "marker";
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.1f;
  marker.scale.y = 0.1f;
  marker.scale.z = 0.1f;
  marker.color.r = 1.;
  marker.color.g = 1.;
  marker.color.b = 0.;
  marker.color.a = 1.;
  for (size_t intersection_id = 0; intersection_id < 3; ++intersection_id) {
    p_from.x = intersected_plane_supports[intersection_id].x();
    p_from.y = intersected_plane_supports[intersection_id].y();
    p_from.z = intersected_plane_supports[intersection_id].z();
    // TODO(fmilano): Double-check!
    p_to.x = intersected_plane_supports[intersection_id].x() +
             intersected_plane_normals[intersection_id].x();
    p_to.y = intersected_plane_supports[intersection_id].y() +
             intersected_plane_normals[intersection_id].y();
    p_to.z = intersected_plane_supports[intersection_id].z() +
             intersected_plane_normals[intersection_id].z();
    marker.points.clear();
    marker.points.push_back(p_from);
    marker.points.push_back(p_to);
    marker.id = intersection_id;
    marker_array.markers.push_back(marker);
  }
  intersection_normals_pub_.publish(marker_array);
  // - Mesh with the interesected faces being colored.
  cgal_msgs::TriangleMesh t_msg;
  cgal_msgs::ColoredMesh c_msg;
  cgal::meshModelToMsg(model_, &t_msg);
  c_msg.mesh = t_msg;
  c_msg.header.frame_id = "marker";
  c_msg.header.stamp = {secs : 0, nsecs : 0};
  c_msg.header.seq = 0;
  std_msgs::ColorRGBA c;
  //   - Color all faces in blue.
  for (size_t i = 0; i < c_msg.mesh.triangles.size(); ++i) {
    c.r = 0.0;
    c.g = 0.0;
    c.b = 1.0;
    c.a = 0.4;
    c_msg.colors.push_back(c);
  }
  //   - Color the intersected faces in red.
  for (const auto &intersected_face_id : intersected_face_ids) {
    c.r = 1.0;
    c.g = 0.0;
    c.b = 0.0;
    c.a = 0.4;
    // find index of triangle with this id
    std::vector<std::string>::iterator it = std::find(
        c_msg.mesh.triangle_ids.begin(), c_msg.mesh.triangle_ids.end(), intersected_face_id);
    CHECK(it != c_msg.mesh.triangle_ids.end())
        << "Unable to find triangle ID " << *it << " in the mesh model.";
    size_t index = std::distance(c_msg.mesh.triangle_ids.begin(), it);
    c_msg.colors[index] = c;
  }
  mesh_with_intersections_pub_.publish(c_msg);

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
  if (!leica_client_["laserOff"].exists()) {
    ROS_ERROR("The service to turn the laser off is not available.");
    return false;
  }
  CHECK(leica_client_["laserOff"].call(empty_srvs.request, empty_srvs.response))
      << "Failed to turn laser off";

  // Optimize for the pose from the marker to the arm base.
  kindr::minimal::QuatTransformation marker_to_armbase_optimized =
      localizer_->optimizeForArmBasePoseInMap(nh_private_.param<bool>("verbose_optimizer", false));
  // Translate the pose in the map into a pose in the map frame.
  kindr::minimal::QuatTransformation map_to_armbase =
      cad_percept::pointlaser_common::getTF(transform_listener_, "map", "marker") *
      marker_to_armbase_optimized;

  // Publish pose of the end effector.
  kindr::minimal::QuatTransformation armbase_to_endeffector = cad_percept::pointlaser_common::getTF(
      transform_listener_, "arm_base", end_effector_topic_name_);
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
  ROS_INFO("Arm base pose in map frame:");
  ROS_INFO_STREAM(map_to_armbase.getPosition().transpose() << "\n");

  // Send corrected pose of the robot base to the controller.
  kindr::minimal::QuatTransformation base_pose_in_world = map_to_armbase * armbase_to_base_;
  ROS_INFO_STREAM("Updated base pose in map, t: "
                  << base_pose_in_world.getPosition().transpose()
                  << ", o: " << base_pose_in_world.getRotation().vector().transpose() << "\n");
  tf::transformKindrToMsg(base_pose_in_world, &response.corrected_base_pose_in_map);

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
  intersection_normals_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>("intersection_normals", 100, true);
  mesh_with_intersections_pub_ =
      nh_private_.advertise<cgal_msgs::ColoredMesh>("mesh_with_intersections", 1, true);
  endeffector_pose_pub_ =
      nh_private_.advertise<geometry_msgs::PoseStamped>("hal_marker_to_end_effector", 1);
  hal_initialize_localization_service_ = nh_.advertiseService(
      "hal_initialize_localization", &MabiLocalizer::initializeHALLocalization, this);
  hal_take_measurement_service_ =
      nh_.advertiseService("hal_take_measurement", &MabiLocalizer::takeMeasurement, this);
  hal_optimize_service_ =
      nh_.advertiseService("hal_optimize", &MabiLocalizer::highAccuracyLocalization, this);
}

}  // namespace pointlaser_loc_ros
}  // namespace cad_percept