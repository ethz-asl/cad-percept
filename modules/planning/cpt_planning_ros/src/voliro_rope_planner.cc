#include <cpt_planning_ros/voliro_rope_planner.h>
#include <glog/logging.h>

namespace cad_percept {
namespace planning {

VoliroRopePlanner::VoliroRopePlanner(ros::NodeHandle nh, ros::NodeHandle nh_private)
  :nh_(nh), nh_private_(nh_private){
  manifold_ = std::make_shared<cad_percept::planning::LinearManifoldInterface>();

  //initi ros interface
  // nh_private_ = nh;
  //pub
  pub_marker_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1, true);
  hose_path_pub = nh_.advertise<nav_msgs::Path>("hose_path", 1000);
  obs_vis_pub = nh_.advertise<visualization_msgs::MarkerArray>("obs_vis", 10);
  pub_trajectory_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("cmd_trajectory", 1);
  pub_mesh_ = cad_percept::MeshModelPublisher(nh, "mesh_3d");
  attraction_pub_ =  nh.advertise<visualization_msgs::Marker>("attraction_vis", 1);
  repulsion_pub_ =  nh.advertise<visualization_msgs::Marker>("repulsion_vis", 1);
  rope_vis_pub_ = nh.advertise<visualization_msgs::Marker>("rope_vis", 1);
  moving_target_pub_ = nh.advertise<geometry_msgs::PoseStamped>("moving_target", 50);

  //sub
  sub_odometry_ = nh_.subscribe("odometry", 1, &VoliroRopePlanner::odometryCallback, this);
  // rope_nodes_sub = nh_.subscribe("rope_vis", 1, &VoliroRopePlanner::ropeUpdateCallback, this);
  moving_target_sub = nh_.subscribe("moving_target", 10, &VoliroRopePlanner::goalCallback, 
                                this);
  joy_sub_ = nh.subscribe<sensor_msgs::Joy>("joy", 10, &VoliroRopePlanner::joyCallback, this);

  //timer
  tf_update_timer_ = nh_.createTimer(ros::Duration(1), &VoliroRopePlanner::tfUpdateCallback,
                                this);  // update TF's every second
  //read params
  readConfig();
  rope_update_timer_ = nh.createTimer(ros::Duration(update_interval_), 
                                      &VoliroRopePlanner::ropeUpdateCallback, this);

  //read mesh model
  loadMesh();
}

void VoliroRopePlanner::resetIntegrator(Eigen::Vector3d start_pos, 
                                        Eigen::Vector3d start_vel){
  integrator.resetTo(start_pos, start_vel);
}

void VoliroRopePlanner::generateTrajectoryOdom_5(){
 
  RMPG::VectorQ target_xyz_1 = goal_a_;
  RMPG::VectorX target_uv_1 = target_xyz_1;

  RMPG::VectorQ target_xyz_2 = goal_b_;
  RMPG::VectorX target_uv_2 = target_xyz_2;

  // RMPG::VectorQ start_xyz = start_;
  // RMPG::VectorX start_uv = start_xyz;

  RMPG::VectorX obs_point_X;

  // set up policies
  Eigen::Matrix3d A{Eigen::Matrix3d::Identity()};

                    // auto pol_2 = std::make_shared<BalancePotential>(target_uv_1, target_uv_2, A);  // 
                    // auto geo_fabric_1 = std::make_shared<AttractionGeometric>(target_uv_1, A);  // 
                    // auto att_1 = std::make_shared<AttractionPotential>(
                    //               (target_uv_1+target_uv_2)*0.5, A);  // 

  std::vector<std::shared_ptr<rmpcpp::PolicyBase<LinSpace>>> policies;
  std::vector<std::shared_ptr<rmpcpp::PolicyBase<LinSpace>>> forcing_policies; 
  // start integrating path
  std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
  bool reached_criteria = false;

  // integrator.resetTo(start_xyz);
  // integrate over trajectory
  // double max_traject_duration_= 1.0;
  for (double t = 0; t < max_traject_duration_; t += dt_) {
    Eigen::Vector3d drone_pos, drone_vel, drone_acc;
    integrator.getState(&drone_pos, &drone_vel, &drone_acc);
                    // std::cout << " root_vel: " << drone_vel(0) <<"; "
                    //           << drone_vel(1) <<"; "
                    //           << drone_vel(2) 
                    //           << std::endl;

                    // add collision avoid policies when needed:
                    // obs point sorted by distance to the drone (shortest distance first)
                    // std::sort( obs_list_.begin( ), obs_list_.end( ), 
                    //   [drone_pos]( const auto& obs_1, const auto& obs_2 )
                    //   {
                    //     return (obs_1-drone_pos).norm() < (obs_2-drone_pos).norm();
                    //   });

    policies.clear();


    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    forcing_policies.clear();
    //speed control test
    Eigen::Vector3d att_pos = target_uv_2+3.0*(drone_pos-target_uv_2).normalized();

    // std::cout<<"[DEBUG] att_pos "<<std::endl;
    // std::cout<< att_pos <<std::endl;
    auto force_policy_spec_1 = std::make_shared<OptimizationPotential>(att_pos, A);  
    forcing_policies.push_back(force_policy_spec_1);

    // get current configuration space position and convert to
    //  task space.
    RMPG::StateQ current_stateQ{drone_pos, drone_vel};
    RMPG::StateX current_stateX = manifold_->convertToX(current_stateQ);
    // evaluate forcing policy and get new accelerations
    std::vector<rmpcpp::PolicyBase<LinSpace>::PValue> evaluated_forc_policies;
    evaluated_forc_policies.resize(forcing_policies.size());
    auto geometry_at_position = manifold_->at(current_stateX);

    for (int i=0; i< forcing_policies.size(); i++){
      geometry_at_position.pull(*forcing_policies[i],&evaluated_forc_policies[i]);
    }

    RMPG::MatrixQ  pulled_M_forc;
    RMPG::VectorQ  pulled_acc_forc;
    RMPG::VectorX  x_to_opt;

    RMPG::MatrixQ sum_ai = RMPG::MatrixQ::Zero();
    RMPG::VectorQ sum_ai_fi = RMPG::VectorQ::Zero();

    for (const auto &RMPBase : evaluated_forc_policies) {
      sum_ai += RMPBase.A_;
      sum_ai_fi += RMPBase.A_ * RMPBase.f_;
    }
    pulled_acc_forc = pinv(sum_ai) * sum_ai_fi;  //weighted average
    pulled_M_forc = sum_ai;
    x_to_opt = drone_pos - att_pos;
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


    // Eigen::Vector3d att_pos = target_uv_2+4.0*(drone_pos-target_uv_2).normalized();
    // if((drone_pos-att_pos).norm()>0.01){
    //   auto att_1 = std::make_shared<AttractionPotential>(att_pos, A);  
    //   policies.push_back(att_1);
    //   //vis attraction
    //   publish_attraction_vis(drone_pos, att_pos);
    // }
    
                    //attraction police is triggered under condition to avoid nonefinite integration
                    // Eigen::Vector3d att_pos = (target_uv_1+target_uv_2)*0.5;
                    // if((drone_pos-att_pos).norm()>0.01){
                    //   auto att_1 = std::make_shared<AttractionPotential>(att_pos, A);  
                    //   policies.push_back(att_1);
                    //   //vis attraction
                    //   publish_attraction_vis(drone_pos, att_pos);
                    // }

                    // if(obs_list_.at(0).norm()<10.0){
                    //   auto geo_fabric_obs = std::make_shared<CollisionAvoidGeometric>(obs_list_.at(0), A);  
                    //   policies.push_back(geo_fabric_obs);
                    //   // auto geo_fabric_obs_2 = std::make_shared<CollisionAvoidGeometric>(obs_list_.at(2), A);  
                    //   // policies.push_back(geo_fabric_obs_2);
                    // }
                    // for(int i=0; i<1;i++){
                    //   auto geo_fabric_obs = std::make_shared<CollisionAvoidGeometric>(obs_list_.at(i), A);  
                    //   policies.push_back(geo_fabric_obs);
                    // }

    // // obs avoid based on rope_sim
    Eigen::Vector3d M_obs_link;
    std::cout<<" push_rope_dir_ "<< push_rope_dir_ <<std::endl;
    std::cout<<" last_push_rope_dir_ "<< last_push_rope_dir_ <<std::endl;

    auto geo_fabric_link = std::make_shared<LinkCollisionAvoidGeometric>(push_rope_dir_, last_push_rope_dir_, A);  
    policies.push_back(geo_fabric_link);

    // for(auto m : ropeVerlet->masses){
    //   auto rope_node_avoid = std::make_shared<CollisionAvoidGeometric>(m->closest_vertex, A, 1.0, 1.0);  
    //   policies.push_back(rope_node_avoid);
    // }

                  // // rope len limit
                  // Eigen::Vector3d M_len_lim;
                  // Eigen::Vector3d rope_seg_1 = target_uv_1-drone_pos;
                  // if(rope_seg_1.norm()>5.0){
                  //   rope_seg_1 = (rope_seg_1.norm()-5.0)*rope_seg_1.normalized();
                  //   auto geo_fabric_lim = std::make_shared<LinkCollisionAvoidGeometric>(rope_seg_1, A);  
                  //   policies.push_back(geo_fabric_lim);
                  //   // std::cout <<"retraction requested"<< std::endl;
                  // }

    //safe dist to the ground
    Eigen::Vector3d ground_point;
    ground_point << drone_pos.x(), drone_pos.y(), -0.1;
    auto safe_dist_ground = std::make_shared<CollisionAvoidGeometric>(ground_point, A, 1.0, 1.0);  
    policies.push_back(safe_dist_ground);

    //baseline
    auto baseline_geom = std::make_shared<BaselineGeometric>(A); 
    policies.push_back(baseline_geom);

                  // obs avoid based on straint rope model-------------------------------------------------
                  // // obs point sorted by distance to the link
                  // auto link_obs_list = obs_list_;
                  // auto another_node_pos = target_uv_2;
                  // double end_drone_dist = (another_node_pos-drone_pos).norm();
                  // std::sort( link_obs_list.begin( ), link_obs_list.end( ), 
                  //   [end_drone_dist, another_node_pos, drone_pos]( const auto& obs_1, const auto& obs_2 )
                  //   {
                  //     double dist_1 = ((another_node_pos-drone_pos).cross(drone_pos-obs_1)).norm()/
                  //                     end_drone_dist;
                  //     double dist_2 = ((another_node_pos-drone_pos).cross(drone_pos-obs_2)).norm()/
                  //                     end_drone_dist;
                  //     return dist_1 < dist_2;
                  //   });

                  // for(auto link_obs : link_obs_list){
                  //   double obs_drone_dist = (link_obs-drone_pos).norm(); 
                  //   double cos_theta = (another_node_pos-drone_pos).dot(link_obs-drone_pos)/
                  //                       (obs_drone_dist*end_drone_dist);
                  //   if(obs_drone_dist<end_drone_dist && cos_theta>0.7){
                  //     auto geo_fabric_link = std::make_shared<LinkCollisionAvoidGeometric>(
                  //                         link_obs_list.at(0), target_uv_2, A);  
                  //     policies.push_back(geo_fabric_link);
                  //     break;
                  //   }
                  // }

                  // // obs point sorted by distance to the link
                  // link_obs_list = obs_list_;
                  // another_node_pos = target_uv_1;
                  // end_drone_dist = (another_node_pos-drone_pos).norm();
                  // std::sort( link_obs_list.begin( ), link_obs_list.end( ), 
                  //   [end_drone_dist, another_node_pos, drone_pos]( const auto& obs_1, const auto& obs_2 )
                  //   {
                  //     double dist_1 = ((another_node_pos-drone_pos).cross(drone_pos-obs_1)).norm()/
                  //                     end_drone_dist;
                  //     double dist_2 = ((another_node_pos-drone_pos).cross(drone_pos-obs_2)).norm()/
                  //                     end_drone_dist;
                  //     return dist_1 < dist_2;
                  //   });

                  // for(auto link_obs : link_obs_list){
                  //   double obs_drone_dist = (link_obs-drone_pos).norm(); 
                  //   double cos_theta = (another_node_pos-drone_pos).dot(link_obs-drone_pos)/
                  //                       (obs_drone_dist*end_drone_dist);
                  //   if(obs_drone_dist<end_drone_dist && cos_theta>0.7){
                  //     auto geo_fabric_link = std::make_shared<LinkCollisionAvoidGeometric>(
                  //                         link_obs_list.at(0), target_uv_1, A);  
                  //     policies.push_back(geo_fabric_link);
                  //     break;
                  //   }
                  // }
                  //-------------------------------------------------------------------------------------------
                
    //get next step
    double vel_desir = 2.0;
    // ROS_WARN("[DEBUG] integrator.integrateStep");

    auto step_result = integrator.integrateStep(policies, 
                                                pulled_M_forc, pulled_acc_forc, x_to_opt, 
                                                manifold_, dt_, vel_desir);

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

  // SurfacePlanner::Result result;
  // result.success = reached_criteria;
  // result.duration = end_time - start_time;
  // std::cout << "Operation took ";
  // display(std::cout, end_time - start_time);
  // std::cout << '\n';
  // std::cout <<"Trajectory Distance:"<<integrator.totalDistance()<<std::endl;
}




void VoliroRopePlanner::publishTrajectory(const mav_msgs::EigenTrajectoryPoint::Vector &trajectory_odom) {
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

void VoliroRopePlanner::publishTrajectory_2() {
  // publish marker message of trajectory
  visualization_msgs::MarkerArray markers;
  double distance = 0.1;  // Distance by which to seperate additional markers. Set 0.0 to disable.
  std::string fram_id = "enu";
  mav_trajectory_generation::drawMavSampledTrajectory(trajectory_odom_, distance,
                                                      fixed_params_.odom_frame, &markers);
  pub_marker_.publish(markers);
  // if (dynamic_params_.output_enable) {
  trajectory_msgs::MultiDOFJointTrajectory msg;
  mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_odom_, &msg);
  msg.header.frame_id = fixed_params_.odom_frame;  ///TODO!!!
  // msg.header.frame_id = fram_id;

  msg.header.stamp = ros::Time::now();
  pub_trajectory_.publish(msg);
  // }
}


std::ostream& VoliroRopePlanner::display(std::ostream& os, std::chrono::nanoseconds ns)
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

nav_msgs::Path VoliroRopePlanner::build_hose_model(std::vector<Eigen::Vector3d> &hose_key_points){
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
void VoliroRopePlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  // ROS_INFO("VoliroRopePlanner::goalCallback");

  // read pos of the new end-effector
  goal_b_(0) = msg->pose.position.x;
  goal_b_(1) = msg->pose.position.y;
  goal_b_(2) = msg->pose.position.z;
  // std::cout <<"----------------"<<std::endl;
  // std::cout <<"goal_b_ updated:"<<goal_b_<<std::endl;

  //update the start pos of the trajectory
  ///use the integrator current state to simulate the start pos,
  ///in practice, replace it to the real state of the drone.
  // if(integrator_init_){
  Eigen::Vector3d drone_pos, drone_vel, drone_acc;
  integrator.getState(&drone_pos, &drone_vel, &drone_acc);
  integrator.resetTo(drone_pos, drone_vel);
  // std::cout <<"Integreator Reset: pos:"<<std::endl;
  // std::cout << drone_pos<<std::endl;
  // std::cout << "vel: "<<std::endl;
  // std::cout << drone_vel <<std::endl;

  // }else{
  //   //put the drone at the start position
  //   //vel = 0
  //   integrator.resetTo(start_);
  //   integrator_init_ = true;
  // }

  //calculate a new trajectory
  trajectory_odom_.clear(); 
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

  //visualize the sphere obs:
  // publish_obs_vis(obs_list_);
}

void VoliroRopePlanner::publish_obs_vis(std::vector<Eigen::Vector3d> &simple_obs_wall) {
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
        p.scale.x = 1.1;
        p.scale.y = 1.1;
        p.scale.z = 1.1;
        p.pose.orientation.w = 1.0;
        p.pose.position.x=obs_point(0);
        p.pose.position.y=obs_point(1);
        p.pose.position.z=obs_point(2);
        p.color.a = 1.0;
        p.color.r = 1.0;
        p.color.g = 0.3;
        p.color.b = 0.0;

        obsArray.markers.push_back(p);
        id++;
    }
    obs_vis_pub.publish(obsArray);
}

void VoliroRopePlanner::publish_attraction_vis(Eigen::Vector3d start, Eigen::Vector3d end){
  //vis push rope arrow----------------------------
   visualization_msgs::Marker marker;
  auto rope_frame = std::string("enu");
  auto rope_time = ros::Time::now();

  marker.type = visualization_msgs::Marker::ARROW;
  marker.id = 0;
  marker.header.frame_id = rope_frame;
  marker.header.stamp = rope_time;

  marker.points.clear();
  geometry_msgs::Point p;
  p.x = start(0);
  p.y = start(1);
  p.z = start(2);

  marker.points.push_back(p);
  p.x = end(0);
  p.y = end(1);
  p.z = end(2);

  marker.points.push_back(p);

  marker.scale.x = 0.03;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.pose.orientation.w = 1.0;

  marker.color.a = 1.0;
  marker.color.r = 0.9;
  marker.color.g = 0.3;
  marker.color.b = 0.3;
  marker.lifetime = ros::Duration(0.5);

  attraction_pub_.publish(marker);
  // id++;
  // std::cout
  // <<"end force:"
  // <<(end-start).norm()
  // <<std::endl;
}

void VoliroRopePlanner::publish_repulsion_vis(Eigen::Vector3d start, Eigen::Vector3d end){
  //vis push rope arrow----------------------------
   visualization_msgs::Marker marker;
  auto rope_frame = std::string("enu");
  auto rope_time = ros::Time::now();

  marker.type = visualization_msgs::Marker::ARROW;
  marker.id = 0;
  marker.header.frame_id = rope_frame;
  marker.header.stamp = rope_time;

  marker.points.clear();
  geometry_msgs::Point p;
  p.x = start(0);
  p.y = start(1);
  p.z = start(2);

  marker.points.push_back(p);
  p.x = end(0);
  p.y = end(1);
  p.z = end(2);

  marker.points.push_back(p);

  marker.scale.x = 0.03;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.pose.orientation.w = 1.0;

  marker.color.a = 1.0;
  marker.color.r = 0.9;
  marker.color.g = 0.3;
  marker.color.b = 0.9;
  marker.lifetime = ros::Duration(0.5);

  repulsion_pub_.publish(marker);
  // id++;
  // std::cout
  // <<"end force:"
  // <<(end-start).norm()
  // <<std::endl;
}


void VoliroRopePlanner::init_obs_wall(){
  // double y=5.0;
  // for(double x = -1.5; x<1.5; x=x+0.5){
  //   for(double z = 0.; z<2.5; z=z+0.5){
  //     Eigen::Vector3d point{x,y,z};
  //     obs_list_.push_back(point);
  //   }
  // }
  obs_list_.push_back({3.0,0.0,2.0});
  // obs_list_.push_back({0.0,5.0,1.5});
}


void VoliroRopePlanner::readConfig() {
  nh_private_.param<std::string>("mesh_path", fixed_params_.mesh_path, "mesh.off");
  nh_private_.param<std::string>("mesh_frame", fixed_params_.mesh_frame, "mesh");
  nh_private_.param<std::string>("body_frame", fixed_params_.body_frame, "imu");
  nh_private_.param<std::string>("enu_frame", fixed_params_.enu_frame, "enu");
  nh_private_.param<std::string>("odom_frame", fixed_params_.odom_frame, "odom");
  nh_private_.param<std::string>("current_reference_frame", fixed_params_.current_reference_frame,
                                 "current_reference");
  
  nh_private_.param("traject_dt", dt_, 0.01);
  nh_private_.param("max_traject_duration", max_traject_duration_, 1.0);
  nh_private_.param<double>("zero_angle", fixed_params_.mesh_zero_angle, 0.0);
  nh_private_.param("rope_safe_dist", rope_safe_dist_, 0.5);

  double zero_x = nh_private_.param("zero_x", 0.0);
  double zero_y = nh_private_.param("zero_y", 0.0);
  double zero_z = nh_private_.param("zero_z", 0.0);
  fixed_params_.mesh_zero = {zero_x, zero_y, zero_z};

  //the start node is fixed
  nh_private_.param("rope_start_x", start_node_pos_(0), -4.0);
  nh_private_.param("rope_start_y", start_node_pos_(1), 0.75);
  nh_private_.param("rope_start_z", start_node_pos_(2), 0.0);
  // the end is able to move 
  nh_private_.param("rope_end_x", end_node_pos_(0), 1.0);
  nh_private_.param("rope_end_y", end_node_pos_(1), 0.75);
  nh_private_.param("rope_end_z", end_node_pos_(2), 0.0);

  //read rope related parameters
  nh_private_.param("pulley_init_x", obj_pos_0_(0), -1.0);
  nh_private_.param("pulley_init_y", obj_pos_0_(1), 0.5);
  nh_private_.param("pulley_init_z", obj_pos_0_(2), -1.0);
  nh_private_.param("pulley_free_yaw", pulley_free_yaw_, true);
  nh_private_.param("rope_node_num", rope_node_num_, 50);
  nh_private_.param("rope_node_mass", rope_node_mass_, float(1.0));
  nh_private_.param("spring_ks", spring_ks_, float(100.0));
  nh_private_.param("spring_rest_len", spring_rest_len_, 0.2);
  nh_private_.param("pulley_friction", pulley_friction_, 0.02);
  nh_private_.param("pulley_radius", pulley_radius_, 0.5);
  nh_private_.param("update_interval", update_interval_, 0.1);
  nh_private_.param("hooked_node_idx", hooked_node_idx_, 30);
  obj_poses_.push_back(obj_pos_0_);

  //init the middel drone
  Eigen::Vector3d init_drone_pos;
  init_drone_pos = 0.5*(start_node_pos_ + end_node_pos_);
  resetIntegrator(init_drone_pos, {0.,0.,0.});
  //init the rope model 
  setTuning({0.7, 13.6, 0.4}, {20.0, 30.0, 0.01}, start_node_pos_, end_node_pos_, 0.01);
}


void VoliroRopePlanner::odometryCallback(const nav_msgs::OdometryConstPtr &odom) {
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
  // if (!dynamic_params_.updateOdomFromCurrentRef) {
    // write transform from odom
    tf::poseMsgToEigen(odom->pose.pose, T_odom_body_);
  // } else {
    // lookup current reference
    // if (!listener_.canTransform(fixed_params_.odom_frame, fixed_params_.current_reference_frame,
    //                             ros::Time(0))) {
    //   ROS_WARN_STREAM("Transform " << fixed_params_.odom_frame << " - "
    //                                << fixed_params_.current_reference_frame << " not available");
    //   return;
    // }
    // // write transform from current reference
    // tf::StampedTransform tf_enu_odom;
    // listener_.lookupTransform(fixed_params_.odom_frame, fixed_params_.current_reference_frame,
    //                           ros::Time(0), tf_enu_odom);
    // tf::transformTFToEigen(tf_enu_odom, T_odom_body_);
  // }

  // if (dynamic_params_.updateOdomVel) {
    // tf::vectorMsgToEigen(odom->twist.twist.linear, v_odom_body_);
  // } else {
    v_odom_body_ = Eigen::Vector3d::Zero();
  // }
  odom_received_ = true;

  //update hooked rope
   Eigen::Vector3d obj_position;
  obj_position(0) = odom->pose.pose.position.x;
  obj_position(1) = odom->pose.pose.position.y;
  obj_position(2) = odom->pose.pose.position.z;
  //  std::cout <<"obj:" << obj_position(0)  << "; "
  //                     << obj_position(1)  << "; "
  //                     << obj_position(2)  << "; "
  //                     << std::endl;
    ///TODO quaternion to angle axis
    //  Eigen::Vector3d obj_orientation;
    //  obj_orientation(0) = odom->pose.pose.orientation.

  //update obj pos in the rope sim
  ropeVerlet->objPosSet(obj_position, pulley_yaw_vec_);

}

void VoliroRopePlanner::tfUpdateCallback(const ros::TimerEvent &event) {
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

  publishMarkers();
}

void VoliroRopePlanner::ropeUpdateCallback(const ros::TimerEvent &event){
  float steps_per_frame = 64.0;
  Eigen::Vector3d gravity(0., 0., -9.8);
  ropeVerlet->mesh_collision_update();
  for (int i = 0; i < steps_per_frame; i++) {
    ropeVerlet->simVerletMovEnd(update_interval_/steps_per_frame, gravity);
    //update pulley free yaw
    // if(pulley_free_yaw){
    //   pulley_yaw_vec = start_node_pos - ropeVerlet->masses.at(0)->position;
    // }
    if(i%8==0){
      ropeVerlet->local_collision_check();
    }
  }
  ropesim::Rope *rope;
  rope = ropeVerlet;

  publish_rope_vis(rope->masses);


    // int min_node_id;
    // for(int node_id = start_idx; node_id<end_idx; node_id++){
    //   Eigen::Vector3d node = rope.at(node_id);
    //   cgal::PointAndPrimitiveId ppid = mesh_model_->getClosestTriangle(node.x(),node.y(),node.z());
    //   Eigen::Vector3d p_on_mesh(ppid.first.x(),ppid.first.y(),ppid.first.z());
    //   Eigen::Vector3d repulsion_vec = node-p_on_mesh;
    //   if(start_idx == 0 && 
    //     repulsion_vec.norm()+(node_id-start_idx)*node_dist < safe_dist){
    //     continue;
    //   }
    //   repulsion_cost = repulsion_cost + repulsion_vec.normalized()*(1/pow(repulsion_vec.norm(),8));
    // }
    // repulsion_cost = repulsion_cost.normalized()*repulsion_cost.norm()/rope.size();


  Eigen::Vector3d repulsion_cost = Eigen::Vector3d::Zero();
  for(auto m:ropeVerlet->masses){
    Eigen::Vector3d repulsion_vec = m->position - m->closest_vertex;
    // std::cout<<" m->closest_vertex "<<m->closest_vertex <<std::endl;

    // if(node_id == 0 && 
    //   repulsion_vec.norm()+(node_id-0)*spring_rest_len_ < rope_safe_dist_){
    //   continue;
    // }
    double repul_norm = repulsion_vec.norm();
    if(repul_norm-repul_norm == 0){
      repulsion_cost +=  repulsion_vec.normalized()*(1/pow(repulsion_vec.norm(),8));
    }
  }
  std::cout<<" repulsion_cost "<<repulsion_cost <<std::endl;

  last_push_rope_dir_ = push_rope_dir_;
  push_rope_dir_ = -1.0*repulsion_cost.normalized()*repulsion_cost.norm()/ropeVerlet->masses.size();;
}

void VoliroRopePlanner::publish_rope_vis(std::vector<ropesim::Mass *> &rope_masses){
  visualization_msgs::Marker rope_marker;
  auto rope_frame = std::string("enu");
  auto rope_time = ros::Time::now();

  rope_marker.type = visualization_msgs::Marker::LINE_LIST;
  // rope_marker.id = id;
  rope_marker.header.frame_id = rope_frame;
  rope_marker.header.stamp = rope_time;

  for (auto &m : rope_masses) {
    // Eigen::Vector3d p = m->position;
    geometry_msgs::Point p;
    p.x = m->position(0);
    p.y = m->position(1);
    p.z = m->position(2);

    rope_marker.scale.x = 0.03;
    rope_marker.scale.y = 0.03;
    rope_marker.scale.z = 0.03;
    rope_marker.pose.orientation.w = 1.0;

    rope_marker.points.push_back(p);
    rope_marker.color.a = 1.0;
    rope_marker.color.r = 1.0;
    rope_marker.color.g = 1.0;
    rope_marker.color.b = 0.0;
    rope_marker.lifetime = ros::Duration(update_interval_*0.9);
  }
  rope_vis_pub_.publish(rope_marker);
  // id++;

  // if(delay>10){
  //publish one end of the rope as a moving target message
  geometry_msgs::PoseStamped mov_target;
  mov_target.header.frame_id = std::string("enu");
  mov_target.header.stamp = ros::Time::now();

  mov_target.pose.position.x = rope_masses.at(0)->position(0);
  mov_target.pose.position.y = rope_masses.at(0)->position(1);
  mov_target.pose.position.z = rope_masses.at(0)->position(2);

  moving_target_pub_.publish(mov_target);
  // delay = 0;
  // }
  // delay++;

}

void VoliroRopePlanner::ropeUpdateCallback(const visualization_msgs::MarkerConstPtr &rope) {
  //find the node hooked by the drone
  auto drone_transformation = T_odom_body_.matrix();
  Eigen::Vector3d drone_pos;
  drone_pos(0) = drone_transformation(0,3);
  drone_pos(1) = drone_transformation(1,3);
  drone_pos(2) = drone_transformation(2,3);

  int min_dist_idx = 0;
  int min_obs_dist_dix = 0;
  int idx = 0;
  double min_dist = 9999.0;
  double min_obs_dist = 9999.0;
  int start_idx = 0;
  //update rope state from the rope_sim
  rope_nodes_vec_.clear();
  double end_obs_dist = 9999.0;
  for (auto node : rope->points) {
    Eigen::Vector3d node_vec;
    node_vec(0) = node.x;
    node_vec(1) = node.y;
    node_vec(2) = node.z;
    rope_nodes_vec_.push_back(node_vec);

    // //find the node closest to the drone
    // double node_drone_dist = (node_vec-drone_pos).norm();
    // if(node_drone_dist<min_dist){
    //   min_dist = node_drone_dist;
    //   min_dist_idx = idx;
    // }

    // //Find the node closest to the external obs
    // //To enable surface operation of the end-effector,
    // //ignore some connected rope nodes for collision check
    // if(idx == 1){
    //   end_obs_dist = (rope_nodes_vec_.at(0)- obs_list_.at(0)).norm();
    //   double interval = (rope_nodes_vec_.at(0)-rope_nodes_vec_.at(1)).norm();
    //   start_idx = 1+(rope_safe_dist_-end_obs_dist)/interval;
    // }
    // if(idx > start_idx){
    //   double node_obs_dist = (node_vec-obs_list_.at(0)).norm();
    //   if(node_obs_dist<min_obs_dist){
    //     min_obs_dist = node_obs_dist;
    //     min_obs_dist_dix = idx;
    //   }
    // }
    // idx++;
  }
  // std::cout<< "hooked_node_idx: "
  //           << min_dist_idx << std::endl;
  // std::cout<< "obs_avoid_node_idx: "
  //           << min_obs_dist_dix << std::endl;

  //check rope collision 
  int min_id;
  Eigen::Vector3d repulsion_cost;
  double node_interval = (rope_nodes_vec_.at(0)-rope_nodes_vec_.at(1)).norm();
  repulsion_cost = path_search_->meshToRopeVec(rope_nodes_vec_, 0, rope_nodes_vec_.size(), node_interval, 1.0);
  publish_repulsion_vis(drone_pos, drone_pos+repulsion_cost);

  //calculate push rope direction on the controlable node
  // push_rope_dir_ = rope_nodes_vec_.at(min_obs_dist_dix) - obs_list_.at(0);
  last_push_rope_dir_ = push_rope_dir_;
  push_rope_dir_ = repulsion_cost;
  // std::cout<< "=============="<< std::endl;
  // std::cout<< "push_rope_dir_: "<< push_rope_dir_ << std::endl;
}

void VoliroRopePlanner::loadMesh() {
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
  // mapping_ = new cad_percept::planning::UVMapping(model_enu_, fixed_params_.mesh_zero,
  //                                                 fixed_params_.mesh_zero_angle);
  // manifold_ = std::make_shared<cad_percept::planning::MeshManifoldInterface>(
  //     model_enu_, fixed_params_.mesh_zero, fixed_params_.mesh_zero_angle);

  // publish
  pub_mesh_.publish(model_enu_, fixed_params_.enu_frame);

  // init path search
  path_search_ = new cad_percept::MeshPathSearch(model_enu_);
  // init rope mand mesh collision model
  ropeVerlet = new ropesim::Rope(end_node_pos_, start_node_pos_, rope_node_num_, rope_node_mass_,
                    spring_ks_, spring_rest_len_, {0,hooked_node_idx_,rope_node_num_-1}, obj_poses_,
                    pulley_friction_, pulley_radius_, model_enu_);


}

void VoliroRopePlanner::publishMarkers() {
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

  // geometry_msgs::Point pt3d;
  // pt3d.x = target_xyz_.x();
  // pt3d.y = target_xyz_.y();
  // pt3d.z = target_xyz_.z();

  visualization_msgs::MarkerArray msg;
  // marker_mesh.points.push_back(pt3d);
  msg.markers.push_back(marker_mesh);
  pub_marker_.publish(msg);
}



void VoliroRopePlanner::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;
  int forth_back = 3;//+ forward , - backward
  int left_right = 2;//+ left, - right
  int up_down = 1; //+ up, - down
  int mode_enable = 4;//BL
  Eigen::Vector3d offset(0., 0., 0.);
  offset(0) += joy->axes[forth_back];
  offset(1) += joy->axes[left_right];
  offset(2) += joy->axes[up_down];
  

  if(joy->buttons[mode_enable]>0){
    //control the motion of the middle drone
    // ropeVerlet->addObjOffset(offset);
    ropeVerlet->addObjOffsetVel(offset*0.001);

  }else{
    //control the motion of end point
    // ropeVerlet->addEndOffset(offset);
    ropeVerlet->addEndOffsetVel(offset*0.001);

  }
  // twist.angular.z = a_scale_*joy->axes[angular_];
  // twist.linear.x = l_scale_*joy->axes[linear_];
  // vel_pub_.publish(twist);
  // id++;
}




















}  // namespace planning
}  // namespace cad_percept