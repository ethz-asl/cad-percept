#include <cpt_planning_ros/omav_planner.h>

namespace cad_percept {
namespace planning {

void OMAVPlanner::runPlanner() {
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
  switch (output_mode_) {
    case OutputMode::Trajectory:
      outputTrajectory(&policies);
      break;
    case OutputMode::Velocity:
      outputVelocities(&policies);
      break;
  }
  std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
  auto duration = end_time - start_time;
  ROS_DEBUG_STREAM("Planner took " << duration.count() << " us");
}

void OMAVPlanner::getPolicyFreeFlight(OMAVPlanner::Policies *policies) {}

void OMAVPlanner::getPolicyTerrain(OMAVPlanner::Policies *policies) {
  Eigen::Matrix3d A{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d B{Eigen::Matrix3d::Identity()};
  A.diagonal() = Eigen::Vector3d({1.0, 1.0, 0.0});
  B.diagonal() = Eigen::Vector3d({0.0, 0.0, 1.0});
  Eigen::Vector3d last_target_uv_;
  double alpha, beta, c;
  auto pol_normal = std::make_shared<TargetPolicy>(last_target_uv_, A, alpha, beta,
                                                   c);  // goes to manifold as quick as possible
  // TargetPolicyPtr pol_along(last_target_uv_, B, alpha_z, beta_z, c_z);  // stays along it
  policies->push_back(pol_normal);
  // policies->push_back(pol_along);
}

void OMAVPlanner::outputVelocities(OMAVPlanner::Policies *policies) {
  PolicyIntegrator integrator;

}

void OMAVPlanner::outputTrajectory(OMAVPlanner::Policies *policies) {}
}  // namespace planning
}  // namespace cad_percept