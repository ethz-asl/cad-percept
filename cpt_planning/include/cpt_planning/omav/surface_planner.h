#ifndef CPT_PLANNING_INCLUDE_CPT_PLANNING_OMAV_SURFACE_PLANNER_H_
#define CPT_PLANNING_INCLUDE_CPT_PLANNING_OMAV_SURFACE_PLANNER_H_
#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_definitions/mesh_model.h>
#include <mav_msgs/conversions.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <ros/ros.h>

#include <Eigen/Dense>
namespace mtg = mav_trajectory_generation;

namespace cad_percept {
namespace planning {
// frames we have here:
// B = Body (that's what the trajectory is following)
// M = Map  (In which the surface is)
// W = World  (in which the trajectory is defined). Assumed to be gravity aligned.
// E = Endeffector (that should go to the surface)

class SurfacePlanner {
 public:
  void planFullContact(const Eigen::Vector3d& p_M, double force,
                       mav_msgs::EigenTrajectoryPoint::Vector* trajectory_sampled,
                       Eigen::Affine3d* contact_pos_w = nullptr);

  void getClosestPointOnMesh(const Eigen::Vector3d& position, Eigen::Vector3d* p_W,
                             Eigen::Vector3d* p_W_normal);

  void InterpolateOrientation(const Eigen::Quaterniond& start_W, const Eigen::Quaterniond& end_W,
                              const uint64_t n_steps, std::vector<Eigen::Quaterniond>* steps);

  void planTrajectory(const Eigen::Affine3d& start_W, const Eigen::Affine3d& end_W,
                      mav_msgs::EigenTrajectoryPoint::Vector* trajectory_sampled);

  void planForce(const Eigen::Vector3d& f_w, const double& duration, const double& ramp_ratio,
                 mav_msgs::EigenTrajectoryPoint::Vector* trajectory_sampled);

  void getT_W_Bintermediate(const Eigen::Vector3d& p_W, const Eigen::Vector3d& p_W_normal,
                            Eigen::Affine3d* T_W_B_contact);

  void getT_W_Bcontact(const Eigen::Vector3d& p_W, const Eigen::Vector3d& p_W_normal,
                       Eigen::Affine3d* T_W_B_contact);

  bool getContactOrientation(const Eigen::Vector3d& normal_W, Eigen::Matrix3d* r_W_E);
  void setStaticFrames(const Eigen::Affine3d& T_B_E, const Eigen::Affine3d& T_W_M);
  void setDynamicFrames(const Eigen::Affine3d& T_W_B,
                        const Eigen::Vector3d& v_W = Eigen::Vector3d::Zero());
  void setLimits(const double v_max, const double a_max) {
    v_max_ = v_max;
    a_max_ = a_max;
  }

  void setMesh(cgal::MeshModel::Ptr model) { model_ = model; }

 protected:
  double sampling_interval_{0.01};
  double v_max_{0.1}, a_max_{0.1};
  Eigen::Affine3d T_B_E_{Eigen::Affine3d::Identity()};  // endeffector to body transform
  Eigen::Affine3d T_W_M_{Eigen::Affine3d::Identity()};  // map to world. assumed static.
  Eigen::Affine3d T_W_B_{Eigen::Affine3d::Identity()};  // world to body, assumed dynamic.
  Eigen::Vector3d v_W_{Eigen::Vector3d::Zero()};        // current velocity
  cgal::MeshModel::Ptr model_;                          // mesh model.
};

}  // namespace planning
}  // namespace cad_percept

#endif  // CPT_PLANNING_INCLUDE_CPT_PLANNING_OMAV_SURFACE_PLANNER_H_
