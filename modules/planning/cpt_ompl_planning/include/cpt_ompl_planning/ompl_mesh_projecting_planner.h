#ifndef CPT_OMPL_PLANNING_OMPL_MESH_PROJECTING_PLANNER_H
#define CPT_OMPL_PLANNING_OMPL_MESH_PROJECTING_PLANNER_H
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

class MeshConstraints : public ob::Constraint {
 public:
  MeshConstraints(cad_percept::cgal::MeshModel::Ptr model) : ob::Constraint(3, 1), model_(model) {}

  void function(const Eigen::Ref<const Eigen::VectorXd> &x,
                Eigen::Ref<Eigen::VectorXd> out) const override {
    // distance to surface
    out[0] = model_->squaredDistance(cad_percept::cgal::Point(x.x(), x.y(), x.z()));
  }

  void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x,
                Eigen::Ref<Eigen::MatrixXd> out) const override {
    auto meshpoint = model_->getClosestTriangle(x.x(), x.y(), x.z());
    Eigen::Vector3d point(x.x(), x.y(), x.z());
    Eigen::Vector3d intersection(meshpoint.first.x(), meshpoint.first.y(), meshpoint.first.z());
    Eigen::Vector3d direction = point - intersection;

    out.leftCols<3>(0) = direction.transpose();
  }

  cad_percept::cgal::MeshModel::Ptr model_;
};

class OMPLMeshProjectingPlanner : public cad_percept::planning::SurfacePlanner {
 public:
  OMPLMeshProjectingPlanner(std::string meshpath, double time);

  inline const std::string getName() const { return "RRTMeshProj" + std::to_string(solve_time_); }

  const SurfacePlanner::Result plan(const Eigen::Vector3d start, const Eigen::Vector3d goal,
                                    std::vector<Eigen::Vector3d> *states_out);

  SurfacePlanner::EdgeList getEdges() const { return rrt_tree_; }

  SurfacePlanner::EdgeList rrt_tree_;
  double solve_time_{1.0};
  cad_percept::cgal::MeshModel::Ptr model_;
};
#endif  // CPT_OMPL_PLANNING_OMPL_MESH_PROJECTING_PLANNER_H
