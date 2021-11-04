#ifndef CPT_PLANNING_IMPLEMENTATION_VOLIRO_ROPE_PLANNER_H
#define CPT_PLANNING_IMPLEMENTATION_VOLIRO_ROPE_PLANNER_H

#include <cpt_planning/interface/linear_manifold_interface.h>
// #include <cpt_planning/interface/surface_planner.h>
#include <cpt_planning_ros/RMPConfigConfig.h>

#include <rmpcpp/core/space.h>
#include <rmpcpp/core/state.h>
// #include <rmpcpp/eval/trapezoidal_integrator.h>
#include <rmpcpp/eval/integrator_with_speed_control.h>

#include <rmpcpp/policies/simple_target_policy.h>
#include <rmpcpp/policies/end_effector_attraction.h>
#include <rmpcpp/policies/acc_based_potential.h>
#include <rmpcpp/policies/acc_potential_dist_balance.h>
#include <rmpcpp/policies/collision_avoid.h>
#include <rmpcpp/policies/link_collision_avoid.h>
#include <rmpcpp/policies/optimization_potential.h>
#include <rmpcpp/policies/baseline_geometry.h>
#include <rmpcpp/policies/ground_lift_geom.h>
#include <rmpcpp/policies/rope_collision.h>


#include <mav_msgs/conversions.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <sensor_msgs/Joy.h>


// add mesh
#include <cgal_definitions/mesh_model.h>
#include <cpt_planning/interface/mesh_manifold_interface.h>
#include <cpt_ros/mesh_model_publisher.h>

// add path search
#include <cpt_ros/mesh_path_search.h>

//rope model
#include <rope.h>


#include <iostream>
#include <tuple>

namespace cad_percept {
namespace planning {

class VoliroRopePlanner{
  // Set up solver
  using RMPG = cad_percept::planning::LinearManifoldInterface;
  using LinSpace = rmpcpp::Space<3>;
  using AttractionPotential = rmpcpp::AccBasedPotential<LinSpace>;
  using BalancePotential = rmpcpp::AccPotentialDistBalance<LinSpace>;
  using AttractionGeometric = rmpcpp::EndEffectorAttraction<LinSpace>;
  using CollisionAvoidGeometric = rmpcpp::CollisionAvoid<LinSpace>;
  using LinkCollisionAvoidGeometric = rmpcpp::LinkCollisionAvoid<LinSpace>;
  // using Integrator = rmpcpp::TrapezoidalIntegrator<rmpcpp::PolicyBase<LinSpace>, RMPG>;
  using Integrator = rmpcpp::IntegratorSpeedControl<rmpcpp::PolicyBase<LinSpace>, RMPG>;

  using OptimizationPotential = rmpcpp::OptimizationPotential<LinSpace>;
  using BaselineGeometric = rmpcpp::BaselineGeometry<LinSpace>;
  using GroundLiftGeometric = rmpcpp::GroundLiftGeom<LinSpace>;
  using RopeCollisionGeom = rmpcpp::RopeColliGeom<LinSpace>;


  struct FixedParams {
    std::string mesh_frame;
    std::string enu_frame;
    std::string odom_frame;
    std::string body_frame;
    std::string current_reference_frame;
    std::string mesh_path;

    Eigen::Vector3d mesh_zero;
    double mesh_zero_angle;
  };

 public:
  VoliroRopePlanner(ros::NodeHandle nh, ros::NodeHandle nh_private);
  VoliroRopePlanner() = delete;

  void generateTrajectoryOdom();
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void init_obs_wall();
  void publish_obs_vis(std::vector<Eigen::Vector3d> &simple_obs_wall);
  void resetIntegrator(Eigen::Vector3d start_pos, Eigen::Vector3d start_vel);

  nav_msgs::Path build_hose_model(std::vector<Eigen::Vector3d> &hose_key_points);

  std::ostream& display(std::ostream& os, std::chrono::nanoseconds ns);
  
  // publishing method
  void publishTrajectory(const mav_msgs::EigenTrajectoryPoint::Vector &trajectory_odom);
  void publishTrajectory_2();
  void publishMarkers();
  void publish_attraction_vis(Eigen::Vector3d start, Eigen::Vector3d end);
  void publish_repulsion_vis(Eigen::Vector3d start, Eigen::Vector3d end);
  void publish_rope_vis(std::vector<ropesim::Mass *> &rope_masses);

  inline const std::string getName() const {
    return "NA";
  }
  
  void setTuning(Eigen::Vector3d tuning_1, Eigen::Vector3d tuning_2,
    Eigen::Vector3d goal_a, Eigen::Vector3d goal_b, double dt = 0.01) {
      tuning_1_ = tuning_1;
      tuning_2_ = tuning_2;
      dt_ = dt;
      goal_a_ = goal_a;
      goal_b_ = goal_b;
  }

 private:
  inline Eigen::Vector3d getVelocityENU() { return T_enu_odom_.rotation() * v_odom_body_; }
  inline Eigen::Vector3d getPositionENU() { return (T_enu_odom_ * T_odom_body_).translation(); }

  // config & startup stuff
  void readConfig();
  void loadMesh();

  // callbacks to update stuff
  void odometryCallback(const nav_msgs::OdometryConstPtr &odom);
  void ropeUpdateCallback(const visualization_msgs::MarkerConstPtr &rope);

  void tfUpdateCallback(const ros::TimerEvent &event);
  void ropeUpdateCallback(const ros::TimerEvent &event);
  void commandUpdateCallback(const ros::TimerEvent &event);
  void envUpdateCallback(const ros::TimerEvent &event);

  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void viconRopeCallback(const nav_msgs::OdometryConstPtr &odom);

  /// Convenience method for pseudo-inverse
  template <int i, int j>
  static inline Eigen::Matrix<double, i, j> pinv(const Eigen::Matrix<double, i, j> &M) {
    return (M.completeOrthogonalDecomposition().pseudoInverse());
  }

  //-------------------------------------------------------
  ros::NodeHandle nh_private_, nh_;

  Eigen::Vector3d tuning_1_, tuning_2_;

  // RMP & Mesh objects
  cad_percept::cgal::MeshModel::Ptr model_enu_;
  cad_percept::planning::UVMapping *mapping_;
  cad_percept::MeshPathSearch *path_search_;

  // std::shared_ptr<cad_percept::planning::MeshManifoldInterface> manifold_;
  std::shared_ptr<cad_percept::planning::LinearManifoldInterface> manifold_;

  // ROS subcribers & publishers
  // pub
  cad_percept::MeshModelPublisher pub_mesh_;
  ros::Publisher pub_marker_;
  ros::Publisher hose_path_pub;
  ros::Publisher obs_vis_pub;
  ros::Publisher pub_trajectory_;  // commanded trajectory
  ros::Publisher attraction_pub_;
  ros::Publisher repulsion_pub_;
  ros::Publisher rope_vis_pub_;
  ros::Publisher moving_target_pub_;

  ros::Publisher policy_vis_pub_;

  
  // sub
  ros::Subscriber sub_odometry_;  // curent odom
  ros::Subscriber vicon_rope_end_odometry_;
  ros::Subscriber rope_nodes_sub;  // curent rope nodes positions
  ros::Subscriber moving_target_sub;
  ros::Subscriber joy_sub_;

  // timer
  ros::Timer tf_update_timer_;
  ros::Timer obs_update_timer_;
  ros::Timer rope_update_timer_;
  ros::Timer command_update_timer_;
  ros::Timer env_update_timer_;


  Eigen::Vector3d start_{0.0, 0.0, 0.0};
  Eigen::Vector3d goal_a_{-0.1, -0.1, -0.1};
  Eigen::Vector3d goal_b_{0.0, 10.0, 2.0};
  Eigen::Vector3d current_pos_{0.0, 0.0, 0.0};
  mav_msgs::EigenTrajectoryPoint::Vector trajectory_odom_;

  Integrator integrator;

  // Configuration
  cpt_planning_ros::RMPConfigConfig dynamic_params_;
  FixedParams fixed_params_;

  // UAV State
  Eigen::Affine3d T_odom_body_, T_enu_odom_, T_enu_mesh_;
  Eigen::Vector3d v_odom_body_;

  tf::TransformListener listener_;
  // rope state
  std::vector<Eigen::Vector3d> rope_nodes_vec_;
  int hooked_node_idx_{0};
  Eigen::Vector3d push_rope_dir_{99.,99.,99.};
  Eigen::Vector3d last_push_rope_dir_{99.,99.,99.};

  //extral constrain for field test
  bool safe_box_constrain_{false};
  bool rope_avoid_constrain_{false};
  bool obs_drone_constrain_{false};

  // Configuration
  bool odom_received_{false};
  bool frames_received_{false};

  double dt_{0.01};                    // [s] temporal resolutions for trajectories
  double max_traject_duration_{1.0};  // [s] amount of lookahead for integration
  double rope_safe_dist_{1.5};
  
  bool mesh_loaded_{false};

  //rope
  ropesim::Rope *ropeVerlet;
  Eigen::Vector3d start_node_pos_{1., 0.75 , 4.};
  Eigen::Vector3d end_node_pos_{-4., 0.75, 4.};
  Eigen::Vector3d obj_pos_0_{-1.5, 0.5, 3.};
  Eigen::Vector3d pulley_yaw_vec_;
  bool pulley_free_yaw_;
  float rope_node_mass_;
  float spring_ks_;
  int rope_node_num_;
  double spring_rest_len_;
  double pulley_friction_;
  double pulley_radius_;
  double update_interval_;//sec
  vector<Eigen::Vector3d> obj_poses_;
  double safe_x_max_, safe_x_min_, safe_y_max_, safe_y_min_, safe_z_max_, safe_z_min_;
  double rope_react_range_, rope_len_lim_, rope_avoid_acc_max_, rope_avoid_range_;
  int node_interval_;

  double vel_desir_;
  double force_sacle_;
  double att_keep_dist_;
  double env_update_interval_;

  bool rope_update_enable_{false};
  bool env_update_enable_{false};
  // bool mesh_loaded_{false};
  double obs_drone_offset_;
  double drone_avoid_acc_max_, drone_avoid_range_;

  double damp_sw_threshold_{1.5};
  double damp_sw_quick_{4.0};
  double damp_B_{1.0};
  
  //policy vis
  double damper_sw_vis_;
  double eng_reg_sw_vis_;


};
}  // namespace planning
}  // namespace cad_percept

#endif  // CPT_PLANNING_IMPLEMENTATION_VOLIRO_ROPE_PLANNER_H
