#ifndef CPT_PLANNING_EVAL_OMPL_MESH_PROJECTING_PLANNER_H
#define CPT_PLANNING_EVAL_OMPL_MESH_PROJECTING_PLANNER_H
#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_definitions/mesh_model.h>
#include <cpt_planning/implementation/rmp_mesh_planner.h>
#include <cpt_planning/interface/surface_planner.h>
#include <ompl/base/Constraint.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <visualization_msgs/MarkerArray.h>
namespace ob = ompl::base;
namespace og = ompl::geometric;

// adopted from https://ompl.kavrakilab.org/ConstrainedPlanningSphere_8cpp_source.html
namespace cad_percept {
namespace planning {

/***
 * Defines constraint function and jacobian
 * for an on-surface constraint.
 */
class MeshProjectionConstraints : public ob::Constraint {
 public:
  MeshProjectionConstraints(cad_percept::cgal::MeshModel::Ptr model)
      : ob::Constraint(3, 1), model_(model) {}

  /***
   * Constraint function (0 if constraint fulfilled).
   * Here it is simply the squared distance to the surface.
   */
  void function(const Eigen::Ref<const Eigen::VectorXd> &x,
                Eigen::Ref<Eigen::VectorXd> out) const override;

  /***
   * Derivative of constraint function
   */
  void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x,
                Eigen::Ref<Eigen::MatrixXd> out) const override;

  cad_percept::cgal::MeshModel::Ptr model_;
};

/***
 * Projecting RRT planner that
 * samples in the R^3 bounding box around the mesh
 * but tries to project any samples onto the surface using
 * the constraint function and derivative
 * defined above.
 */
class OMPLMeshProjectingPlanner : public cad_percept::planning::SurfacePlanner {
 public:
  /***
   * Sets up the planner
   * @param meshpath Path to mesh file (.off)
   * @param time Solve Time in seconds.
   */
  OMPLMeshProjectingPlanner(std::string meshpath, double time);

  inline const std::string getName() const {
    return "RRTMeshProj" + std::to_string((int)(solve_time_ * 1000.0));
  }

  const SurfacePlanner::Result plan(const Eigen::Vector3d start, const Eigen::Vector3d goal,
                                    std::vector<Eigen::Vector3d> *states_out);

  /***
   * Returns all sampled edges in the tree.
   */
  SurfacePlanner::EdgeList getEdges() const { return rrt_tree_; }

  SurfacePlanner::EdgeList rrt_tree_;
  double solve_time_{1.0};
  cad_percept::cgal::MeshModel::Ptr model_;
};
}  // namespace planning
}  // namespace cad_percept
#endif  // CPT_PLANNING_EVAL_OMPL_MESH_PROJECTING_PLANNER_H
