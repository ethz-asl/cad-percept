#include <cpt_planning/implementation/rmp_linear_planner.h>
#include <glog/logging.h>

namespace cad_percept {
namespace planning {

RMPLinearPlanner::RMPLinearPlanner(Eigen::Vector3d tuning_1,
                               Eigen::Vector3d tuning_2)
    : tuning_1_(tuning_1), tuning_2_(tuning_2) {
//   cad_percept::cgal::MeshModel::create(mesh_path, &model_, true);
//   Eigen::Vector3d zero(0.0, 0.0, 0.0);
//   double zero_angle = 0;
//   mapping_ = new cad_percept::planning::UVMapping(model_, zero, zero_angle);
  manifold_ =
      std::make_shared<cad_percept::planning::LinearManifoldInterface>();

}

const SurfacePlanner::Result RMPLinearPlanner::plan(const Eigen::Vector3d start,
                                                  const Eigen::Vector3d goal,
                                                  std::vector<Eigen::Vector3d> *states_out) {
  // Set up solver
  using RMPG = cad_percept::planning::LinearManifoldInterface;
  using LinSpace = rmpcpp::Space<3>;
  using TargetPolicy = rmpcpp::SimpleTargetPolicy<LinSpace>;
  using Integrator = rmpcpp::TrapezoidalIntegrator<TargetPolicy, RMPG>;

  RMPG::VectorQ target_xyz = goal;
  RMPG::VectorX target_uv = target_xyz;

  RMPG::VectorQ start_xyz = start;
  RMPG::VectorX start_uv = start_xyz;
  Integrator integrator;

  // set up policies
  Eigen::Matrix3d A{Eigen::Matrix3d::Identity()};
//   Eigen::Matrix3d B{Eigen::Matrix3d::Identity()};
//   A.diagonal() = Eigen::Vector3d({1.0, 1.0, 0.0});
//   B.diagonal() = Eigen::Vector3d({0.0, 0.0, 1.0});

  auto pol_2 = std::make_shared<TargetPolicy>(target_uv, A, tuning_1_[0], tuning_1_[1],
                                              tuning_1_[2]);  // goes to target
//   auto pol_3 =
//       std::make_shared<TargetPolicy>(Eigen::Vector3d::Zero(), B, tuning_2_[0], tuning_2_[1],
//                                      tuning_2_[2]);  // stays on surface

  std::vector<std::shared_ptr<TargetPolicy>> policies;
  policies.push_back(pol_2);
//   policies.push_back(pol_3);

  // start integrating path
  std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
  bool reached_criteria = false;

  integrator.resetTo(start_xyz);
  for (double t = 0; t < 500.0; t += dt_) {
    auto integrator_state = integrator.integrateStep(policies, manifold_, dt_);
    states_out->push_back(integrator_state.position);

    if (integrator.atRest(0.01, 0.01)) {
      reached_criteria = true;
      LOG(INFO) << "Residual position after integration: "
                << (integrator_state.position - target_xyz).norm();
      break;
    }
  }
  std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();

  SurfacePlanner::Result result;
  result.success = reached_criteria;
  result.duration = end_time - start_time;
  return result;
}


//generate trajectory using RMP Simple TargetPolicy:
void RMPLinearPlanner::generateTrajectoryOdom(const Eigen::Vector3d start,
                                              const Eigen::Vector3d goal,
                                         mav_msgs::EigenTrajectoryPoint::Vector *trajectory_odom){
   // Set up solver
  using RMPG = cad_percept::planning::LinearManifoldInterface;
  using LinSpace = rmpcpp::Space<3>;
  using TargetPolicy = rmpcpp::SimpleTargetPolicy<LinSpace>;
  using Integrator = rmpcpp::TrapezoidalIntegrator<TargetPolicy, RMPG>;

  RMPG::VectorQ target_xyz = goal;
  RMPG::VectorX target_uv = target_xyz;

  RMPG::VectorQ start_xyz = start;
  RMPG::VectorX start_uv = start_xyz;
  Integrator integrator;

  // set up policies
  Eigen::Matrix3d A{Eigen::Matrix3d::Identity()};
//   Eigen::Matrix3d B{Eigen::Matrix3d::Identity()};
//   A.diagonal() = Eigen::Vector3d({1.0, 1.0, 0.0});
//   B.diagonal() = Eigen::Vector3d({0.0, 0.0, 1.0});

  auto pol_2 = std::make_shared<TargetPolicy>(target_uv, A, tuning_1_[0], tuning_1_[1],
                                              tuning_1_[2]);  // goes to target
//   auto pol_3 =
//       std::make_shared<TargetPolicy>(Eigen::Vector3d::Zero(), B, tuning_2_[0], tuning_2_[1],
//                                      tuning_2_[2]);  // stays on surface

  std::vector<std::shared_ptr<TargetPolicy>> policies;
  policies.push_back(pol_2);
//   policies.push_back(pol_3);

  // start integrating path
  std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
  bool reached_criteria = false;

  integrator.resetTo(start_xyz);
    // integrate over trajectory
  double max_traject_duration_= 500.0;
  for (double t = 0; t < max_traject_duration_; t += dt_) {
    auto step_result = integrator.integrateStep(policies, manifold_, dt_);

    if (!step_result.allFinite()) {
      ROS_WARN("Error, nonfinite integration data, stopping integration");
      trajectory_odom->clear();
      return;
    }

    // get 3D trajectory point in ENU
    mav_msgs::EigenTrajectoryPoint pt_test;
    pt_test.time_from_start_ns = t * 1e9;
    integrator.getState(&pt_test.position_W, &pt_test.velocity_W, &pt_test.acceleration_W);

    // convert point to odom with current transform
    // mav_msgs::EigenTrajectoryPoint pt_odom = pt_enu;
    // keep current orientation
    // pt_odom.orientation_W_B = T_odom_body_.rotation();

    trajectory_odom->push_back(pt_test);

    if (integrator.atRest()) {
      ROS_DEBUG_STREAM("Integrator finished after " << t << " s with a distance of "
                                                    << integrator.totalDistance());
      break;
    }
  }
}


//generate trajectory using Optimization Fabrics: acceleration based potentials:
void RMPLinearPlanner::generateTrajectoryOdom_2(const Eigen::Vector3d start,
                                              const Eigen::Vector3d goal,
                                         mav_msgs::EigenTrajectoryPoint::Vector *trajectory_odom){
   // Set up solver
  using RMPG = cad_percept::planning::LinearManifoldInterface;
  using LinSpace = rmpcpp::Space<3>;
  using TargetPolicy = rmpcpp::AccBasedPotential<LinSpace>;
  using Integrator = rmpcpp::TrapezoidalIntegrator<TargetPolicy, RMPG>;

  RMPG::VectorQ target_xyz = goal;
  RMPG::VectorX target_uv = target_xyz;

  RMPG::VectorQ start_xyz = start;
  RMPG::VectorX start_uv = start_xyz;
  Integrator integrator;

  // set up policies
  Eigen::Matrix3d A{Eigen::Matrix3d::Identity()};
//   Eigen::Matrix3d B{Eigen::Matrix3d::Identity()};
//   A.diagonal() = Eigen::Vector3d({1.0, 1.0, 0.0});
//   B.diagonal() = Eigen::Vector3d({0.0, 0.0, 1.0});

  auto pol_2 = std::make_shared<TargetPolicy>(target_uv, A);  // goes to target

//   auto pol_3 =
//       std::make_shared<TargetPolicy>(Eigen::Vector3d::Zero(), B, tuning_2_[0], tuning_2_[1],
//                                      tuning_2_[2]);  // stays on surface

  std::vector<std::shared_ptr<TargetPolicy>> policies;
  policies.push_back(pol_2);
//   policies.push_back(pol_3);

  // start integrating path
  std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
  bool reached_criteria = false;

  integrator.resetTo(start_xyz);
    // integrate over trajectory
  double max_traject_duration_= 500.0;
  for (double t = 0; t < max_traject_duration_; t += dt_) {
    auto step_result = integrator.integrateStep(policies, manifold_, dt_);

    if (!step_result.allFinite()) {
      ROS_WARN("Error, nonfinite integration data, stopping integration");
      trajectory_odom->clear();
      return;
    }

    // get 3D trajectory point in ENU
    mav_msgs::EigenTrajectoryPoint pt_test;
    pt_test.time_from_start_ns = t * 1e9;
    integrator.getState(&pt_test.position_W, &pt_test.velocity_W, &pt_test.acceleration_W);

    // convert point to odom with current transform
    // mav_msgs::EigenTrajectoryPoint pt_odom = pt_enu;
    // keep current orientation
    // pt_odom.orientation_W_B = T_odom_body_.rotation();

    trajectory_odom->push_back(pt_test);

    if (integrator.atRest()) {
      ROS_DEBUG_STREAM("Integrator finished after " << t << " s with a distance of "
                                                    << integrator.totalDistance());
      break;
    }
  }
}


void RMPLinearPlanner::generateTrajectoryOdom_3(const Eigen::Vector3d start,
                                              const Eigen::Vector3d goal_1,
                                              const Eigen::Vector3d goal_2,
                                         mav_msgs::EigenTrajectoryPoint::Vector *trajectory_odom){
   // Set up solver
  using RMPG = cad_percept::planning::LinearManifoldInterface;
  using LinSpace = rmpcpp::Space<3>;
  using BalancePotential = rmpcpp::AccPotentialDistBalance<LinSpace>;
  using AttractionGeometric = rmpcpp::EndEffectorAttraction<LinSpace>;
  using Integrator = rmpcpp::TrapezoidalIntegrator<rmpcpp::PolicyBase<LinSpace>, RMPG>;

  RMPG::VectorQ target_xyz_1 = goal_1;
  RMPG::VectorX target_uv_1 = target_xyz_1;

  RMPG::VectorQ target_xyz_2 = goal_2;
  RMPG::VectorX target_uv_2 = target_xyz_2;

  RMPG::VectorQ start_xyz = start;
  RMPG::VectorX start_uv = start_xyz;
  Integrator integrator;

  // set up policies
  Eigen::Matrix3d A{Eigen::Matrix3d::Identity()};

  auto pol_2 = std::make_shared<BalancePotential>(target_uv_1, target_uv_2, A);  // 
  auto geo_fabric_1 = std::make_shared<AttractionGeometric>(target_uv_1, A);  // 


  std::vector<std::shared_ptr<rmpcpp::PolicyBase<LinSpace>>> policies;
  policies.push_back(pol_2);
  policies.push_back(geo_fabric_1);

  // start integrating path
  std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
  bool reached_criteria = false;

  integrator.resetTo(start_xyz);
    // integrate over trajectory
  double max_traject_duration_= 500.0;
  for (double t = 0; t < max_traject_duration_; t += dt_) {
    auto step_result = integrator.integrateStep(policies, manifold_, dt_);

    if (!step_result.allFinite()) {
      ROS_WARN("Error, nonfinite integration data, stopping integration");
      trajectory_odom->clear();
      return;
    }

    // get 3D trajectory point in ENU
    mav_msgs::EigenTrajectoryPoint pt_test;
    pt_test.time_from_start_ns = t * 1e9;
    integrator.getState(&pt_test.position_W, &pt_test.velocity_W, &pt_test.acceleration_W);

    // convert point to odom with current transform
    // mav_msgs::EigenTrajectoryPoint pt_odom = pt_enu;
    // keep current orientation
    // pt_odom.orientation_W_B = T_odom_body_.rotation();

    trajectory_odom->push_back(pt_test);

    if (integrator.atRest()) {
      ROS_DEBUG_STREAM("Integrator finished after " << t << " s with a distance of "
                                                    << integrator.totalDistance());
      break;
    }
  }
}



void RMPLinearPlanner::publishTrajectory(const mav_msgs::EigenTrajectoryPoint::Vector &trajectory_odom) {
  // publish marker message of trajectory
  visualization_msgs::MarkerArray markers;
  double distance = 0.1;  // Distance by which to seperate additional markers. Set 0.0 to disable.
  std::string fram_id = "enu";
  mav_trajectory_generation::drawMavSampledTrajectory(trajectory_odom, distance,
                                                      fram_id, &markers);
  pub_marker_.publish(markers);

  // if (dynamic_params_.output_enable) {
  //   trajectory_msgs::MultiDOFJointTrajectory msg;
  //   mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_odom, &msg);
  //   msg.header.frame_id = fixed_params_.odom_frame;
  //   msg.header.stamp = ros::Time::now();
  //   pub_trajectory_.publish(msg);
  // }
}



}  // namespace planning
}  // namespace cad_percept