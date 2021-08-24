#ifndef CPT_PLANNING_IMPLEMENTATION_RMP_LINEAR_PLANNER_H
#define CPT_PLANNING_IMPLEMENTATION_RMP_LINEAR_PLANNER_H
// #include <cgal_definitions/cgal_typedefs.h>
// #include <cgal_definitions/mesh_model.h>
// #include <cpt_planning/coordinates/face_coords.h>
// #include <cpt_planning/coordinates/uv_mapping.h>
// #include <cpt_planning/interface/mesh_manifold_interface.h>
#include <cpt_planning/interface/linear_manifold_interface.h>
#include <cpt_planning/interface/surface_planner.h>
#include <rmpcpp/core/space.h>
#include <rmpcpp/core/state.h>
#include <rmpcpp/eval/trapezoidal_integrator.h>
#include <rmpcpp/policies/simple_target_policy.h>
#include <rmpcpp/policies/end_effector_attraction.h>
#include <rmpcpp/policies/acc_based_potential.h>
#include <rmpcpp/policies/acc_potential_dist_balance.h>
#include <rmpcpp/policies/collision_avoid.h>
#include <rmpcpp/policies/link_collision_avoid.h>


#include <mav_msgs/conversions.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>


#include <iostream>
namespace cad_percept {
namespace planning {

/*
 * Simplified planner with default tuning that
 * implements the "LinearPlanner" interface used for
 * automated comparison
 */
class RMPLinearPlanner : public SurfacePlanner {
 public:
  RMPLinearPlanner(Eigen::Vector3d tuning_1 = {0.6, 5.8, 0.56}, 
                    Eigen::Vector3d tuning_2 = {6.0, 8.0, 0.14});
//   RMPLinearPlanner(std::string mesh_path, Eigen::Vector3d tuning_1 = {0.6, 5.8, 0.56},
                //  Eigen::Vector3d tuning_2 = {6.0, 8.0, 0.14});

  const SurfacePlanner::Result plan(const Eigen::Vector3d start, const Eigen::Vector3d goal,
                                    std::vector<Eigen::Vector3d> *states_out);

  void init_ros_interface(ros::NodeHandle nh){
      pub_marker_ = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1, true);
      hose_path_pub = nh.advertise<nav_msgs::Path>("hose_path", 1000);
      obs_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("obs_vis", 10);


  }

  void generateTrajectoryOdom(const Eigen::Vector3d start,
                              const Eigen::Vector3d goal,
                              mav_msgs::EigenTrajectoryPoint::Vector *trajectory_odom);
  void generateTrajectoryOdom_2(const Eigen::Vector3d start,
                              const Eigen::Vector3d goal,
                              mav_msgs::EigenTrajectoryPoint::Vector *trajectory_odom);
  void generateTrajectoryOdom_3(const Eigen::Vector3d start,
                              const Eigen::Vector3d goal_1,
                              const Eigen::Vector3d goal_2,
                              mav_msgs::EigenTrajectoryPoint::Vector *trajectory_odom);

  void generateTrajectoryOdom_4(const Eigen::Vector3d start,
                              const Eigen::Vector3d goal_1,
                              const Eigen::Vector3d goal_2,
                              const std::vector<Eigen::Vector3d> &obs_list,
                              mav_msgs::EigenTrajectoryPoint::Vector *trajectory_odom);

  void generateTrajectoryOdom_5();

  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

  void init_obs_wall();

  void publish_obs_vis(std::vector<Eigen::Vector3d> &simple_obs_wall);
  
  void resetIntegrator(Eigen::Vector3d start_pos, Eigen::Vector3d start_vel);

  nav_msgs::Path build_hose_model(std::vector<Eigen::Vector3d> &hose_key_points);



  std::ostream& display(std::ostream& os, std::chrono::nanoseconds ns);
  
  // publishing method
  void publishTrajectory(const mav_msgs::EigenTrajectoryPoint::Vector &trajectory_odom);
  void publishTrajectory_2();


  inline const std::string getName() const {
    return "NA";
  }
  
  void setTuning(Eigen::Vector3d tuning_1, Eigen::Vector3d tuning_2, double dt = 0.01) {
    tuning_1_ = tuning_1;
    tuning_2_ = tuning_2;
    dt_ = dt;
  }

 private:
//   void storeMapping() const;

  Eigen::Vector3d tuning_1_, tuning_2_;
  double dt_{0.01};

//   cad_percept::cgal::MeshModel::Ptr model_;
//   cad_percept::planning::UVMapping *mapping_;
  std::shared_ptr<cad_percept::planning::LinearManifoldInterface> manifold_;
  
  ros::Publisher pub_marker_;
  ros::Publisher hose_path_pub;
  ros::Publisher obs_vis_pub;



  Eigen::Vector3d start_{0.0, 0.0, 0.0};
  Eigen::Vector3d goal_a_{-0.1, -0.1, -0.1};
  Eigen::Vector3d goal_b_{0.0, 10.0, 2.0};
  Eigen::Vector3d current_pos_{0.0, 0.0, 0.0};
  std::vector<Eigen::Vector3d> obs_list_;
  mav_msgs::EigenTrajectoryPoint::Vector trajectory_odom_;


    // Set up solver
  using RMPG = cad_percept::planning::LinearManifoldInterface;
  using LinSpace = rmpcpp::Space<3>;
  using AttractionPotential = rmpcpp::AccBasedPotential<LinSpace>;
  using BalancePotential = rmpcpp::AccPotentialDistBalance<LinSpace>;
  using AttractionGeometric = rmpcpp::EndEffectorAttraction<LinSpace>;
  using CollisionAvoidGeometric = rmpcpp::CollisionAvoid<LinSpace>;
  using LinkCollisionAvoidGeometric = rmpcpp::LinkCollisionAvoid<LinSpace>;
  using Integrator = rmpcpp::TrapezoidalIntegrator<rmpcpp::PolicyBase<LinSpace>, RMPG>;
  Integrator integrator;

  bool integrator_init_{false};


};
}  // namespace planning
}  // namespace cad_percept

#endif  // CPT_PLANNING_IMPLEMENTATION_RMP_LINEAR_PLANNER_H
