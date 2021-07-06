#include <cpt_planning_ros/omav_planner.h>
#include <eigen_conversions/eigen_msg.h>

namespace cad_percept {
namespace planning {

OMAVPlanner::OMAVPlanner(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : nh_(nh), nh_private_(nh_private) {
  pub_mesh_ = cad_percept::MeshModelPublisher(nh, "mesh_3d");
  pub_trajectory_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("cmd_trajectory", 1);
  pub_marker_ = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1, true);

  readConfig();
  server_.setCallback(boost::bind(&OMAVPlanner::configCallback, this, _1, _2));

  loadMesh();

  sub_joystick_ = nh.subscribe("/joy", 1, &OMAVPlanner::joystickCallback, this);
  sub_odometry_ = nh.subscribe("odometry", 1, &OMAVPlanner::odometryCallback, this);
  tf_update_timer_ = nh.createTimer(ros::Duration(1), &OMAVPlanner::tfUpdateCallback,
                                    this);  // update TF's every second
}

void OMAVPlanner::runPlanner() {
  if (!allInitialized()) {
    ROS_WARN_STREAM("Not planning, node not fully initalized, is planner zeroed?");
    return;
  }

  std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
  Policies policies;

  // Populate Policies
  switch (flight_mode_) {
    case FlightMode::FreeFlight:
      getPolicyFreeFlight(&policies);
      break;
    case FlightMode::Terrain:
      getPolicyTerrain(&policies);
      break;
    default:
      ROS_WARN_STREAM("Not planning, unsupported flight mode");
      return;
      break;
  }

  // Output to controller
  if (output_mode_ == OutputMode::Trajectory) {
    mav_msgs::EigenTrajectoryPoint::Vector traject_odom;
    generateTrajectoryOdom(policies, &traject_odom);
    publishMarkers();
    publishTrajectory(traject_odom);
  } else if (output_mode_ == OutputMode::Velocity) {
    ROS_WARN("Velocity mode not implemented");
  }
  std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
  auto duration = end_time - start_time;
  ROS_DEBUG_STREAM("Planner took " << duration.count() << " us");
}

void OMAVPlanner::readConfig() {
  nh_private_.param<std::string>("mesh_path", fixed_params_.mesh_path, "mesh.off");
  nh_private_.param<std::string>("mesh_frame", fixed_params_.mesh_frame, "mesh");
  nh_private_.param<std::string>("body_frame", fixed_params_.body_frame, "imu");
  nh_private_.param<std::string>("enu_frame", fixed_params_.enu_frame, "enu");
  nh_private_.param<std::string>("odom_frame", fixed_params_.odom_frame, "odom");

  nh_private_.param<double>("zero_angle", fixed_params_.mesh_zero_angle, 0.0);

  double zero_x = nh_private_.param("zero_x", 0.0);
  double zero_y = nh_private_.param("zero_y", 0.0);
  double zero_z = nh_private_.param("zero_z", 0.0);
  fixed_params_.mesh_zero = {zero_x, zero_y, zero_z};
}

void OMAVPlanner::loadMesh() {
  mesh_loaded_ = cad_percept::cgal::MeshModel::create(fixed_params_.mesh_path, &model_enu_, true);

  if (!mesh_loaded_) {
    // Kill if mesh couldn't be loaded.
    ROS_FATAL_STREAM("Could not load mesh " << fixed_params_.mesh_path);
    exit(-100);
  }

  // get transformation to ENU
  while (!listener_.waitForTransform(fixed_params_.enu_frame, fixed_params_.mesh_frame,
                                     ros::Time(0), ros::Duration(0.1)) &&
         ros::ok()) {
    ROS_WARN_STREAM("Waiting for transform " << fixed_params_.enu_frame << " - "
                                             << fixed_params_.mesh_frame);
  }
  tf::StampedTransform tf_enu_mesh;
  listener_.lookupTransform(fixed_params_.enu_frame, fixed_params_.mesh_frame, ros::Time(0),
                            tf_enu_mesh);
  tf::transformTFToEigen(tf_enu_mesh, T_enu_mesh_);

  // Transform mesh to ENU frame
  model_enu_->transform(
      cad_percept::cgal::eigenTransformationToCgalTransformation(T_enu_mesh_.matrix()));

  // Creating mappings and manifold interface
  mapping_ = new cad_percept::planning::UVMapping(model_enu_, fixed_params_.mesh_zero,
                                                  fixed_params_.mesh_zero_angle);
  manifold_ = std::make_shared<cad_percept::planning::MeshManifoldInterface>(
      model_enu_, fixed_params_.mesh_zero, fixed_params_.mesh_zero_angle);

  // publish
  pub_mesh_.publish(model_enu_, fixed_params_.enu_frame);
}

void OMAVPlanner::getPolicyFreeFlight(OMAVPlanner::Policies *policies) {
  auto pol_freeflight = std::make_shared<TargetPolicy>(
      target_xyz_, Eigen::Matrix3d::Identity(), dynamic_params_.freeflight_alpha,
      dynamic_params_.freeflight_beta, dynamic_params_.freeflight_gamma);
  policies->push_back(pol_freeflight);

  if (dynamic_params_.obstacle_enable) {
    Eigen::Vector3d target, gradient;  // todo: get closest obstacle and gradient
    auto pol_col_avoidance = std::make_shared<RepulsivePolicy>(
        target, gradient, dynamic_params_.obstacle_nu_rep, dynamic_params_.obstacle_eta_damp,
        dynamic_params_.obstacle_nu_damp, dynamic_params_.obstacle_eta_damp,
        dynamic_params_.obstacle_gamma_damp, dynamic_params_.obstacle_r);
    policies->push_back(pol_col_avoidance);
  }
}

void OMAVPlanner::getPolicyTerrain(OMAVPlanner::Policies *policies) {
  Eigen::Matrix3d A_along{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d A_normal{Eigen::Matrix3d::Identity()};
  A_along.diagonal() = Eigen::Vector3d({1.0, 1.0, 0.0});
  A_normal.diagonal() = Eigen::Vector3d({0.0, 0.0, 1.0});

  // policy that effects along the surface
  auto pol_along = std::make_shared<TargetPolicy>(
      target_uvh_, A_along, dynamic_params_.terrain_alpha_along, dynamic_params_.terrain_beta_along,
      dynamic_params_.terrain_gamma_along);

  // policy that effects normal to the surface
  auto pol_normal = std::make_shared<TargetPolicy>(
      target_uvh_, A_normal, dynamic_params_.terrain_alpha_normal,
      dynamic_params_.terrain_beta_normal, dynamic_params_.terrain_gamma_normal);

  policies->push_back(pol_normal);
  policies->push_back(pol_along);
}

void OMAVPlanner::generateVelocitiesOdom(const OMAVPlanner::Policies &policies,
                                         Eigen::Vector3d *velocity) {
  // set up integrator and run for one step
  PolicyIntegrator integrator_enu;
  integrator_enu.resetTo(getPositionENU(), getVelocityENU());
  auto step_result = integrator_enu.integrateStep(policies, manifold_, dt_);

  // check result before sending to system!
  if (step_result.allFinite()) {
    *velocity = step_result.velocity;
  } else {
    ROS_WARN("Error, nonfinite integration data, stopping integration");
    *velocity = Eigen::Vector3d::Zero();  // 0 Velocity probably the safest default.
  }
}

void OMAVPlanner::generateTrajectoryOdom(const OMAVPlanner::Policies &policies,
                                         mav_msgs::EigenTrajectoryPoint::Vector *trajectory_odom) {
  // setup integrator
  PolicyIntegrator integrator_enu;
  integrator_enu.resetTo(getPositionENU(), getVelocityENU());

  // integrate over trajectory
  for (double t = 0; t < max_traject_duration_; t += dt_) {
    auto step_result = integrator_enu.integrateStep(policies, manifold_, dt_);

    if (!step_result.allFinite()) {
      ROS_WARN("Error, nonfinite integration data, stopping integration");
      trajectory_odom->clear();
      return;
    }

    // get 3D trajectory point in ENU
    mav_msgs::EigenTrajectoryPoint pt_enu;
    pt_enu.time_from_start_ns = t * 1e9;
    integrator_enu.getState(&pt_enu.position_W, &pt_enu.velocity_W, &pt_enu.acceleration_W);

    // convert point to odom with current transform
    mav_msgs::EigenTrajectoryPoint pt_odom = T_enu_odom_.inverse() * pt_enu;

    // keep current orientation
    pt_odom.orientation_W_B = T_odom_body_.rotation();

    trajectory_odom->push_back(pt_odom);

    if (integrator_enu.atRest()) {
      ROS_DEBUG_STREAM("Integrator finished after " << t << " s with a distance of "
                                                    << integrator_enu.totalDistance());
      break;
    }
  }
}

void OMAVPlanner::publishTrajectory(const mav_msgs::EigenTrajectoryPoint::Vector &trajectory_odom) {
  // publish marker message of trajectory
  visualization_msgs::MarkerArray markers;
  double distance = 0.1;  // Distance by which to seperate additional markers. Set 0.0 to disable.
  mav_trajectory_generation::drawMavSampledTrajectory(trajectory_odom, distance,
                                                      fixed_params_.odom_frame, &markers);
  pub_marker_.publish(markers);

  if (dynamic_params_.output_enable) {
    trajectory_msgs::MultiDOFJointTrajectory msg;
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_odom, &msg);
    msg.header.frame_id = fixed_params_.odom_frame;
    msg.header.stamp = ros::Time::now();
    pub_trajectory_.publish(msg);
  }
}

void OMAVPlanner::configCallback(cpt_planning_ros::RMPConfigConfig &config, uint32_t level){
  if(level != 0){
    //avoid this global callback
    return ;
  }
  dynamic_params_ = config;

}

void OMAVPlanner::publishMarkers() {
  visualization_msgs::Marker marker_mesh;
  marker_mesh.header.frame_id = fixed_params_.enu_frame;
  marker_mesh.header.stamp = ros::Time();
  marker_mesh.ns = "meshmarker";
  marker_mesh.id = 0;
  marker_mesh.type = visualization_msgs::Marker::SPHERE_LIST;
  marker_mesh.action = visualization_msgs::Marker::ADD;
  marker_mesh.pose.position.x = 0.0;
  marker_mesh.pose.position.y = 0.0;
  marker_mesh.pose.position.z = 0.0;
  marker_mesh.pose.orientation.x = 0.0;
  marker_mesh.pose.orientation.y = 0.0;
  marker_mesh.pose.orientation.z = 0.0;
  marker_mesh.pose.orientation.w = 1.0;
  marker_mesh.scale.x = 0.25;
  marker_mesh.scale.y = 0.25;
  marker_mesh.scale.z = 0.25;
  marker_mesh.color.a = 1.0;  // Don't forget to set the alpha!
  marker_mesh.color.r = 0.0;
  marker_mesh.color.g = 1.0;
  marker_mesh.color.b = 0.0;

  geometry_msgs::Point pt3d;
  pt3d.x = target_xyz_.x();
  pt3d.y = target_xyz_.y();
  pt3d.z = target_xyz_.z();

  visualization_msgs::MarkerArray msg;
  msg.markers.push_back(marker_mesh);
  pub_marker_.publish(msg);
}

void OMAVPlanner::joystickCallback(const sensor_msgs::JoyConstPtr &joy) {
  if (joy->buttons[5]) {
    target_uvh_ = mapping_->point3DtoUVH((T_enu_odom_ * T_odom_body_).translation());
    ROS_INFO_STREAM("Reset setpoint to current position");
    zeroed_ = true;
  }

  target_uvh_.x() += joy->axes[2] * dynamic_params_.joystick_xy_scaling;
  target_uvh_.y() += -joy->axes[3] * dynamic_params_.joystick_xy_scaling;
  target_uvh_.z() += joy->axes[5] * dynamic_params_.joystick_z_scaling;
  target_uvh_.z() = std::clamp(target_uvh_.z(), dynamic_params_.terrain_min_dist,
                               dynamic_params_.terrain_max_dist);

  // clamp to manifold (avoid going out of the map)
  target_uvh_.topRows<2>() =
      (Eigen::Vector2d)mapping_->clipToManifold((Eigen::Vector2d)target_uvh_.topRows<2>());

  // convert to xyz target
  target_xyz_ = mapping_->pointUVHto3D(target_uvh_);

  // for each input, run the planner
  // this should probably be changed for velocity mode!
  runPlanner();
}

void OMAVPlanner::odometryCallback(const nav_msgs::OdometryConstPtr &odom) {
  if (odom->header.frame_id != fixed_params_.odom_frame) {
    ROS_WARN_STREAM("Odom frame name mismatch, msg: " << odom->header.frame_id
                                                      << " / cfg: " << fixed_params_.odom_frame);
    return;
  }

  /**
   * For some systems we want to plan from current odom,
   * for others from current reference.
   * We still might want to get the current velocity,
   * so we leave the user the choice to get that via odom (as tf, _i think_ doesnt have velocities)
   * pain.
   */
  if (!dynamic_params_.updateOdomFromCurrentRef) {
    // write transform from odom
    tf::poseMsgToEigen(odom->pose.pose, T_odom_body_);
  } else {
    // lookup current reference
    if (!listener_.canTransform(fixed_params_.odom_frame, fixed_params_.current_reference_frame,
                                ros::Time(0))) {
      ROS_WARN_STREAM("Transform " << fixed_params_.odom_frame << " - "
                                   << fixed_params_.current_reference_frame << " not available");
      return;
    }
    // write transform from current reference
    tf::StampedTransform tf_enu_odom;
    listener_.lookupTransform(fixed_params_.odom_frame, fixed_params_.current_reference_frame,
                              ros::Time(0), tf_enu_odom);
    tf::transformTFToEigen(tf_enu_odom, T_odom_body_);
  }

  if (dynamic_params_.updateOdomVel) {
    tf::vectorMsgToEigen(odom->twist.twist.linear, v_odom_body_);
  } else {
    v_odom_body_ = Eigen::Vector3d::Zero();
  }
  odom_received_ = true;
}

void OMAVPlanner::tfUpdateCallback(const ros::TimerEvent &event) {
  if (!listener_.canTransform(fixed_params_.enu_frame, fixed_params_.odom_frame, ros::Time(0))) {
    ROS_WARN_STREAM("Transform " << fixed_params_.enu_frame << " - " << fixed_params_.odom_frame
                                 << " not available");
    return;
  }

  // write transform
  tf::StampedTransform tf_enu_odom;
  listener_.lookupTransform(fixed_params_.enu_frame, fixed_params_.odom_frame, ros::Time(0),
                            tf_enu_odom);
  tf::transformTFToEigen(tf_enu_odom, T_enu_odom_);

  frames_received_ = true;
}

}  // namespace planning
}  // namespace cad_percept