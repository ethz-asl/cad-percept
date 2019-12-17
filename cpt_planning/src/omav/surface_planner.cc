#include <cpt_planning/omav/surface_planner.h>
namespace cad_percept {
namespace planning {

void SurfacePlanner::getClosestPointOnMesh(const Eigen::Vector3d& position_M, Eigen::Vector3d* p_W,
                                            Eigen::Vector3d* p_W_normal) {
  cgal::PointAndPrimitiveId point =
      model_->getClosestTriangle(position_M.x(), position_M.y(), position_M.z());
  ROS_WARN("Got closest triangle");
  cgal::Vector normal_cgal = model_->getNormal(point);
  ROS_WARN("Got normal");
  Eigen::Vector3d p_M, p_M_normal;
  p_M << point.first.x(), point.first.y(), point.first.z();
  p_M_normal << normal_cgal.x(), normal_cgal.y(), normal_cgal.z();

  *p_W = T_W_M_ * p_M;
  *p_W_normal = (T_W_M_.rotation() * p_M_normal).normalized();
}

void SurfacePlanner::planFullContact(const Eigen::Vector3d& p_W, double force,
                     mav_msgs::EigenTrajectoryPoint::Vector* trajectory_sampled) {
  // get normal,force and position on mesh
  Eigen::Affine3d T_W_B_contact, T_W_B_intermediate;
  Eigen::Vector3d normal_W, position_W;
  getClosestPointOnMesh(p_W, &position_W, &normal_W);
  Eigen::Vector3d force_W = -normal_W * force;

  // get intermediate points
  getT_W_Bintermediate(position_W, normal_W, &T_W_B_intermediate);
  getT_W_Bcontact(position_W, normal_W, &T_W_B_contact);

  // plan engage, contact and retract
  planTrajectory(T_W_B_, T_W_B_intermediate, trajectory_sampled);
  planTrajectory(T_W_B_intermediate, T_W_B_contact, trajectory_sampled);
  planForce(force_W, 5.0, 0.15, trajectory_sampled);
  planTrajectory(T_W_B_contact, T_W_B_intermediate, trajectory_sampled);
}

void SurfacePlanner::InterpolateOrientation(const Eigen::Quaterniond& start,
                                            const Eigen::Quaterniond& end, const uint64_t n_steps,
                                            std::vector<Eigen::Quaterniond>* steps) {
  steps->clear();
  steps->resize(n_steps);

  for (uint64_t i = 0; i < n_steps; ++i) {
    double t = (i + 1) / (double)n_steps;
    steps->at(i) = start.slerp(t, end);
  }
}

void SurfacePlanner::planTrajectory(const Eigen::Affine3d& start, const Eigen::Affine3d& end,
                                    mav_msgs::EigenTrajectoryPoint::Vector* trajectory_sampled) {
  mav_msgs::EigenTrajectoryPoint::Vector trajectory_sampled_part;
  /* Setup trajectory generation */
  mtg::NonlinearOptimizationParameters parameters;
  parameters.max_iterations = 1000;
  parameters.f_rel = 0.05;
  parameters.x_rel = 0.1;
  parameters.time_penalty = 500.0;
  parameters.initial_stepsize_rel = 0.1;
  parameters.inequality_constraint_tolerance = 0.1;

  mav_trajectory_generation::Vertex::Vector vertices;
  const int dimension = 3;
  const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
  mav_trajectory_generation::Vertex start_vertex(dimension), end_vertex(dimension);

  // add start/end constraints
  Eigen::Quaterniond start_orientation(start.linear());
  Eigen::Quaterniond end_orientation(end.linear());

  start_vertex.makeStartOrEnd(start.translation(), derivative_to_optimize);
  start_vertex.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, v_W_);
  end_vertex.makeStartOrEnd(end.translation(), derivative_to_optimize);

  vertices.push_back(start_vertex);
  vertices.push_back(end_vertex);
  std::vector<double> segment_times = estimateSegmentTimes(vertices, v_max_, a_max_);

  // solve polynomials
  const int N = 10;
  mtg::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, v_max_);
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION,
                                    a_max_);
  opt.optimize();

  mav_trajectory_generation::Trajectory trajectory;
  opt.getTrajectory(&trajectory);

  mtg::sampleWholeTrajectory(trajectory, sampling_interval_, &trajectory_sampled_part);

  // fix orientations
  std::vector<Eigen::Quaterniond> orientations_interpolated;
  InterpolateOrientation(start_orientation, end_orientation, trajectory_sampled_part.size(),
                         &orientations_interpolated);

  //  hack to get orientations.
  for (uint64_t i = 0; i < trajectory_sampled_part.size(); ++i) {
    trajectory_sampled_part.at(i).orientation_W_B = orientations_interpolated.at(i);
  }

  // add to trajectory
  int64_t last_element_time = 0;
  if (trajectory_sampled->size() > 0) {
    last_element_time = trajectory_sampled->back().time_from_start_ns;
  }

  for (uint64_t i = 0; i < trajectory_sampled_part.size(); ++i) {
    // get last point
    mav_msgs::EigenTrajectoryPoint new_point(trajectory_sampled_part.at(i));
    new_point.time_from_start_ns += last_element_time;
    trajectory_sampled->push_back(new_point);
  }
}

void SurfacePlanner::planForce(const Eigen::Vector3d& f_w, const double& duration,
                               const double& ramp_ratio,
                               mav_msgs::EigenTrajectoryPoint::Vector* trajectory_sampled) {
  if (trajectory_sampled->size() == 0) {
    ROS_WARN_STREAM("Could not add force to empty trajectory.");
    return;
  }
  if (ramp_ratio > 0.5 || ramp_ratio < 0.0) {
    ROS_WARN_STREAM("Invalid ramp ratio." << ramp_ratio);
    return;
  }

  int ramp_steps = (ramp_ratio * duration) / sampling_interval_;
  int full_steps = ((1 - 2 * ramp_ratio) * duration) / sampling_interval_;

  // get copy of last waypoint
  auto last_waypoint(trajectory_sampled->back());
  Eigen::Affine3d last_T_W_B(last_waypoint.orientation_W_B);
  last_T_W_B.translation() = last_waypoint.position_W;

  int64_t curr_end_time = last_waypoint.time_from_start_ns;
  int64_t timestep = static_cast<int64_t>(1E9 * sampling_interval_);

  // sinusoidal interpolation during ramp_ratio % of time
  for (int i = 0; i < ramp_steps; i++) {
    auto force_waypoint(last_waypoint);
    force_waypoint.force_W = f_w * sin(M_PI_2 * static_cast<double>(i) / ramp_steps);
    force_waypoint.time_from_start_ns = (curr_end_time + timestep);
    curr_end_time += timestep;
    trajectory_sampled->push_back(force_waypoint);

    if (i % 10) {
      ROS_WARN_STREAM(i << " w: " << force_waypoint.force_W << ", B"
                        << last_T_W_B.rotation().transpose() * force_waypoint.force_W);
    }
  }

  // sinusoidal interpolation during ramp_ratio % of time
  for (int i = 0; i < full_steps; i++) {
    auto force_waypoint(last_waypoint);
    force_waypoint.force_W = f_w;
    force_waypoint.time_from_start_ns = (curr_end_time + timestep);
    curr_end_time += timestep;
    trajectory_sampled->push_back(force_waypoint);
    if (i % 10) {
      ROS_WARN_STREAM(i << " w: " << force_waypoint.force_W << ", B"
                        << last_T_W_B.rotation().transpose() * force_waypoint.force_W);
    }
  }

  // reverse interpolation
  for (int i = 0; i < ramp_steps; i++) {
    auto force_waypoint(last_waypoint);
    force_waypoint.force_W = f_w * sin(M_PI_2 * (1.0 - (static_cast<double>(i) / ramp_steps)));
    force_waypoint.time_from_start_ns = (curr_end_time + timestep);
    curr_end_time += timestep;
    trajectory_sampled->push_back(force_waypoint);
    if (i % 10) {
      ROS_WARN_STREAM(i << " w: " << force_waypoint.force_W << ", B"
                        << last_T_W_B.rotation().transpose() * force_waypoint.force_W);
    }
  }
  // get last position in trajectory
}

void SurfacePlanner::getT_W_Bintermediate(const Eigen::Vector3d& p_W,
                                          const Eigen::Vector3d& p_W_normal,
                                          Eigen::Affine3d* T_W_B_contact) {
  Eigen::Matrix3d r_W_E;
  getContactOrientation(p_W_normal, &r_W_E);

  Eigen::Affine3d T_W_E_contact;
  T_W_E_contact.linear() = r_W_E;
  T_W_E_contact.translation() = p_W + p_W_normal * 0.75;

  *T_W_B_contact = T_W_E_contact * T_B_E_.inverse();
}

void SurfacePlanner::getT_W_Bcontact(const Eigen::Vector3d& p_W, const Eigen::Vector3d& p_W_normal,
                                     Eigen::Affine3d* T_W_B_contact) {
  // get endeffector orientation in world frame
  Eigen::Matrix3d r_W_E;
  getContactOrientation(p_W_normal, &r_W_E);

  // get resulting Body pose in world frame
  Eigen::Affine3d T_W_E_contact;
  T_W_E_contact.linear() = r_W_E;
  T_W_E_contact.translation() = p_W + p_W_normal * 0.01;  // leave 2 cm distance

  *T_W_B_contact = T_W_E_contact * T_B_E_.inverse();
}

bool SurfacePlanner::getContactOrientation(const Eigen::Vector3d& normal_W,
                                           Eigen::Matrix3d* r_W_E) {
  // Gravity in E
  Eigen::Vector3d gravity_W = -Eigen::Vector3d::UnitZ();
  Eigen::Vector3d last_normal_W = normal_W;

  // first, check that we are not too close to singularity.
  if ((-last_normal_W).cross(gravity_W).norm() < 0.005) {
    ROS_WARN("In or close to singularity");
    return false;
  }

  // x = along gravity
  // y = perpendicular to gravity and z
  // z= - normal
  r_W_E->col(1) = -(-last_normal_W).cross(gravity_W);
  r_W_E->col(1).normalize();

  r_W_E->col(2) = -last_normal_W;
  r_W_E->col(2).normalize();

  r_W_E->col(0) = r_W_E->col(2).cross(r_W_E->col(1));
  r_W_E->col(0).normalize();

  // check right handed-ness
  if (r_W_E->determinant() < 0) {
    r_W_E->col(0) = -r_W_E->col(0);
  }

  return true;
}

void SurfacePlanner::setStaticFrames(const Eigen::Affine3d& T_B_E, const Eigen::Affine3d& T_W_M) {
  T_B_E_ = T_B_E;
  T_W_M_ = T_W_M;
}

void SurfacePlanner::setDynamicFrames(const Eigen::Affine3d& T_W_B, const Eigen::Vector3d& v_W) {
  T_W_B_ = T_W_B;
  v_W_ = v_W;
}

}  // namespace planning
}  // namespace cad_percept