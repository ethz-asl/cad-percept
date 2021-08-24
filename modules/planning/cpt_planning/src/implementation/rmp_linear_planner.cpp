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
 
void RMPLinearPlanner::generateTrajectoryOdom_4(const Eigen::Vector3d start,
                            const Eigen::Vector3d goal_1,
                            const Eigen::Vector3d goal_2,
                            const std::vector<Eigen::Vector3d> &obs_list,
                            mav_msgs::EigenTrajectoryPoint::Vector *trajectory_odom){
   // Set up solver
  using RMPG = cad_percept::planning::LinearManifoldInterface;
  using LinSpace = rmpcpp::Space<3>;
  using BalancePotential = rmpcpp::AccPotentialDistBalance<LinSpace>;
  using AttractionGeometric = rmpcpp::EndEffectorAttraction<LinSpace>;
  using CollisionAvoidGeometric = rmpcpp::CollisionAvoid<LinSpace>;
  using Integrator = rmpcpp::TrapezoidalIntegrator<rmpcpp::PolicyBase<LinSpace>, RMPG>;

  RMPG::VectorQ target_xyz_1 = goal_1;
  RMPG::VectorX target_uv_1 = target_xyz_1;

  RMPG::VectorQ target_xyz_2 = goal_2;
  RMPG::VectorX target_uv_2 = target_xyz_2;

  RMPG::VectorQ start_xyz = start;
  RMPG::VectorX start_uv = start_xyz;

  RMPG::VectorX obs_point_X;

  Integrator integrator;

  // set up policies
  Eigen::Matrix3d A{Eigen::Matrix3d::Identity()};

  auto pol_2 = std::make_shared<BalancePotential>(target_uv_1, target_uv_2, A);  // 
  auto geo_fabric_1 = std::make_shared<AttractionGeometric>(target_uv_1, A);  // 


  std::vector<std::shared_ptr<rmpcpp::PolicyBase<LinSpace>>> policies;
  policies.push_back(pol_2);
  policies.push_back(geo_fabric_1);
  // add collision avoid policies:
  for(auto obs_point : obs_list){
    //add one collision avoid geometric fabric for each sensed obs_point
    obs_point_X = obs_point;
    auto geo_fabric_obs = std::make_shared<CollisionAvoidGeometric>(obs_point_X, A);  
    policies.push_back(geo_fabric_obs);
  }


  // start integrating path
  std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
  bool reached_criteria = false;

  integrator.resetTo(start_xyz);
    // integrate over trajectory
  double max_traject_duration_= 60.0;
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
      std::cout <<"Integrator finished after " << t << " s with a distance of "
                                                    << integrator.totalDistance()
                                                    << std::endl;
      break;
    }
  }
  std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();

  SurfacePlanner::Result result;
  result.success = reached_criteria;
  result.duration = end_time - start_time;
  std::cout << "Operation took ";
  display(std::cout, end_time - start_time);
  std::cout << '\n';
  std::cout <<"Trajectory Distance:"<<integrator.totalDistance()<<std::endl;
}

void RMPLinearPlanner::resetIntegrator(Eigen::Vector3d start_pos, 
                                        Eigen::Vector3d start_vel){
  integrator.resetTo(start_pos, start_vel);
}

void RMPLinearPlanner::generateTrajectoryOdom_5(){
 
  RMPG::VectorQ target_xyz_1 = goal_a_;
  RMPG::VectorX target_uv_1 = target_xyz_1;

  RMPG::VectorQ target_xyz_2 = goal_b_;
  RMPG::VectorX target_uv_2 = target_xyz_2;

  // RMPG::VectorQ start_xyz = start_;
  // RMPG::VectorX start_uv = start_xyz;

  RMPG::VectorX obs_point_X;

  // set up policies
  Eigen::Matrix3d A{Eigen::Matrix3d::Identity()};

  auto pol_2 = std::make_shared<BalancePotential>(target_uv_1, target_uv_2, A);  // 
  auto geo_fabric_1 = std::make_shared<AttractionGeometric>(target_uv_1, A);  // 
  auto att_1 = std::make_shared<AttractionPotential>(
                (target_uv_1+target_uv_2)/2.0, A);  // 

  std::vector<std::shared_ptr<rmpcpp::PolicyBase<LinSpace>>> policies;

  // start integrating path
  std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
  bool reached_criteria = false;

  // integrator.resetTo(start_xyz);
    // integrate over trajectory
  double max_traject_duration_= 1.0;
  for (double t = 0; t < max_traject_duration_; t += dt_) {
    Eigen::Vector3d drone_pos, drone_vel, drone_acc;
    integrator.getState(&drone_pos, &drone_vel, &drone_acc);
    std::cout << " root_vel: " << drone_vel(0) <<"; "
              << drone_vel(1) <<"; "
              << drone_vel(2) 
              << std::endl;

    // add collision avoid policies when needed:
    // obs point sorted by distance to the drone (shortest distance first)
    std::sort( obs_list_.begin( ), obs_list_.end( ), 
      [drone_pos]( const auto& obs_1, const auto& obs_2 )
      {
        return (obs_1-drone_pos).norm() < (obs_2-drone_pos).norm();
      });

    policies.clear();
    policies.push_back(att_1);
    // policies.push_back(pol_2);
    // policies.push_back(geo_fabric_1);

    if(obs_list_.at(0).norm()<10.0){
      auto geo_fabric_obs = std::make_shared<CollisionAvoidGeometric>(obs_list_.at(0), A);  
      policies.push_back(geo_fabric_obs);
      auto geo_fabric_obs_2 = std::make_shared<CollisionAvoidGeometric>(obs_list_.at(2), A);  
      policies.push_back(geo_fabric_obs_2);
    }
    // for(int i=0; i<1;i++){
    //   auto geo_fabric_obs = std::make_shared<CollisionAvoidGeometric>(obs_list_.at(i), A);  
    //   policies.push_back(geo_fabric_obs);
    // }

    // obs point sorted by distance to the link
    auto link_obs_list = obs_list_;
    auto another_node_pos = target_uv_2;
    double end_drone_dist = (another_node_pos-drone_pos).norm();
    std::sort( link_obs_list.begin( ), link_obs_list.end( ), 
      [end_drone_dist, another_node_pos, drone_pos]( const auto& obs_1, const auto& obs_2 )
      {
        double dist_1 = ((another_node_pos-drone_pos).cross(drone_pos-obs_1)).norm()/
                        end_drone_dist;
        double dist_2 = ((another_node_pos-drone_pos).cross(drone_pos-obs_2)).norm()/
                        end_drone_dist;
        return dist_1 < dist_2;
      });

    for(auto link_obs : link_obs_list){
      double obs_drone_dist = (link_obs-drone_pos).norm(); 
      double cos_theta = (another_node_pos-drone_pos).dot(link_obs-drone_pos)/
                          (obs_drone_dist*end_drone_dist);
      if(obs_drone_dist<end_drone_dist && cos_theta>0.7){
        auto geo_fabric_link = std::make_shared<LinkCollisionAvoidGeometric>(
                            link_obs_list.at(0), target_uv_2, A);  
        policies.push_back(geo_fabric_link);
        break;
      }
    }


    // obs point sorted by distance to the link
    link_obs_list = obs_list_;
    another_node_pos = target_uv_1;
    end_drone_dist = (another_node_pos-drone_pos).norm();
    std::sort( link_obs_list.begin( ), link_obs_list.end( ), 
      [end_drone_dist, another_node_pos, drone_pos]( const auto& obs_1, const auto& obs_2 )
      {
        double dist_1 = ((another_node_pos-drone_pos).cross(drone_pos-obs_1)).norm()/
                        end_drone_dist;
        double dist_2 = ((another_node_pos-drone_pos).cross(drone_pos-obs_2)).norm()/
                        end_drone_dist;
        return dist_1 < dist_2;
      });

    for(auto link_obs : link_obs_list){
      double obs_drone_dist = (link_obs-drone_pos).norm(); 
      double cos_theta = (another_node_pos-drone_pos).dot(link_obs-drone_pos)/
                          (obs_drone_dist*end_drone_dist);
      if(obs_drone_dist<end_drone_dist && cos_theta>0.7){
        auto geo_fabric_link = std::make_shared<LinkCollisionAvoidGeometric>(
                            link_obs_list.at(0), target_uv_1, A);  
        policies.push_back(geo_fabric_link);
        break;
      }
    }
  




    //get next step
    auto step_result = integrator.integrateStep(policies, manifold_, dt_);

    if (!step_result.allFinite()) {
      ROS_WARN("Error, nonfinite integration data, stopping integration");
      trajectory_odom_.clear();
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

    trajectory_odom_.push_back(pt_test);

    if (integrator.atRest()) {
      std::cout <<"Integrator finished after " << t << " s with a distance of "
                                                    << integrator.totalDistance()
                                                    << std::endl;
      break;
    }
  }
  std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();

  SurfacePlanner::Result result;
  result.success = reached_criteria;
  result.duration = end_time - start_time;
  std::cout << "Operation took ";
  display(std::cout, end_time - start_time);
  std::cout << '\n';
  std::cout <<"Trajectory Distance:"<<integrator.totalDistance()<<std::endl;
}




void RMPLinearPlanner::publishTrajectory(const mav_msgs::EigenTrajectoryPoint::Vector &trajectory_odom) {
  // publish marker message of trajectory
  visualization_msgs::MarkerArray markers;
  double distance = 0.1;  // Distance by which to seperate additional markers. Set 0.0 to disable.
  std::string fram_id = "enu";
  mav_trajectory_generation::drawMavSampledTrajectory(trajectory_odom_, distance,
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

void RMPLinearPlanner::publishTrajectory_2() {
  // publish marker message of trajectory
  visualization_msgs::MarkerArray markers;
  double distance = 0.1;  // Distance by which to seperate additional markers. Set 0.0 to disable.
  std::string fram_id = "enu";
  mav_trajectory_generation::drawMavSampledTrajectory(trajectory_odom_, distance,
                                                      fram_id, &markers);
  pub_marker_.publish(markers);
}


std::ostream& RMPLinearPlanner::display(std::ostream& os, std::chrono::nanoseconds ns)
{
    using namespace std;
    using namespace std::chrono;
    typedef duration<int, ratio<86400>> days;
    char fill = os.fill();
    os.fill('0');
    auto d = duration_cast<days>(ns);
    ns -= d;
    auto h = duration_cast<hours>(ns);
    ns -= h;
    auto m = duration_cast<minutes>(ns);
    ns -= m;
    auto s = duration_cast<seconds>(ns);
    ns -= s;
    auto ms = duration_cast<milliseconds>(ns);
    os << setw(2) << d.count() << "d:"
       << setw(2) << h.count() << "h:"
       << setw(2) << m.count() << "m:"
       << setw(2) << s.count() << "s:"
       << setw(2) << ms.count() << "ms";
    os.fill(fill);
    return os;
};

nav_msgs::Path RMPLinearPlanner::build_hose_model(std::vector<Eigen::Vector3d> &hose_key_points){
    // Circle parameters
    nav_msgs::Path waypoints;
    geometry_msgs::PoseStamped pt;

    waypoints.header.frame_id = std::string("enu");
    waypoints.header.stamp = ros::Time::now();
    pt.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    // pt.header.frame_id = std::string("enu");
    // pt.header.stamp = ros::Time::now();
    for(auto keypoint : hose_key_points){
      pt.pose.position.x =  keypoint(0);
      pt.pose.position.y =  keypoint(1);
      pt.pose.position.z =  keypoint(2);
      waypoints.poses.push_back(pt);    
    } 
    // Return
    return waypoints;
}

//read new goal and calculate the trajectory
void RMPLinearPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  ROS_INFO("RMPLinearPlanner::goalCallback");

  // read pos of the new end-effector
  goal_b_(0) = msg->pose.position.x;
  goal_b_(1) = msg->pose.position.y;
  goal_b_(2) = msg->pose.position.z;
  std::cout <<"----------------"<<std::endl;
  std::cout <<"goal_b_ updated:"<<goal_b_<<std::endl;

  //update the start pos of the trajectory
  ///use the integrator current state to simulate the start pos,
  ///in practice, replace it to the real state of the drone.
  if(integrator_init_){
    Eigen::Vector3d drone_pos, drone_vel, drone_acc;
    integrator.getState(&drone_pos, &drone_vel, &drone_acc);
    integrator.resetTo(drone_pos, drone_vel);
    std::cout <<"Integreator Reset: pos:"<<std::endl;
    std::cout << drone_pos<<std::endl;
    std::cout << "vel: "<<std::endl;
    std::cout << drone_vel <<std::endl;

  }else{
    //put the drone at the start position
    //vel = 0
    integrator.resetTo(start_);
    integrator_init_ = true;
  }

  //calculate a new trajectory
  generateTrajectoryOdom_5();

  //visualize the new trajectory
  publishTrajectory_2();

  //visualize the hose:
  nav_msgs::Path hose_path;
  current_pos_ = trajectory_odom_.back().position_W;
  start_ = current_pos_;
  std::vector<Eigen::Vector3d> hose_key_points;
  hose_key_points.push_back(goal_a_);
  hose_key_points.push_back(current_pos_);
  hose_key_points.push_back(goal_b_);
  hose_path = build_hose_model(hose_key_points);
  hose_path_pub.publish(hose_path);

  //visualize the obs:
  publish_obs_vis(obs_list_);
}

void RMPLinearPlanner::publish_obs_vis(std::vector<Eigen::Vector3d> &simple_obs_wall) {
    visualization_msgs::MarkerArray obsArray;
    auto obs_frame = std::string("enu");
    auto obs_time = ros::Time::now();

    int id = 0;
    for (auto obs_point : simple_obs_wall) {
        visualization_msgs::Marker p;
        p.type = visualization_msgs::Marker::SPHERE;
        p.id = id;
        p.header.frame_id = obs_frame;
        p.header.stamp = obs_time;
        p.scale.x = 0.1;
        p.scale.y = 0.1;
        p.scale.z = 0.1;
        p.pose.orientation.w = 1.0;
        p.pose.position.x=obs_point(0);
        p.pose.position.y=obs_point(1);
        p.pose.position.z=obs_point(2);
        p.color.a = 1.0;
        p.color.r = 1.0;
        p.color.g = 1.0;
        p.color.b = 0.0;

        obsArray.markers.push_back(p);
        id++;
    }
    obs_vis_pub.publish(obsArray);
}



void RMPLinearPlanner::init_obs_wall(){
  double y=5.0;
  for(double x = -1.5; x<1.5; x=x+0.5){
    for(double z = 0.; z<2.5; z=z+0.5){
      Eigen::Vector3d point{x,y,z};
      obs_list_.push_back(point);
    }
  }
}


}  // namespace planning
}  // namespace cad_percept