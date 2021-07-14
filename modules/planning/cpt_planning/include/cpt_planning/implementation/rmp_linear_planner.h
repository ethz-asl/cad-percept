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

#include <mav_msgs/conversions.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>


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
  }

  void generateTrajectoryOdom(const Eigen::Vector3d start,
                              const Eigen::Vector3d goal,
                              mav_msgs::EigenTrajectoryPoint::Vector *trajectory_odom);

  // publishing method
  void publishTrajectory(const mav_msgs::EigenTrajectoryPoint::Vector &trajectory_odom);


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


};
}  // namespace planning
}  // namespace cad_percept

#endif  // CPT_PLANNING_IMPLEMENTATION_RMP_LINEAR_PLANNER_H
