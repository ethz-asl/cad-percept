#ifndef CPT_CHOMP_PLANNING_CHOMP_MESH_PLANNER_H
#define CPT_CHOMP_PLANNING_CHOMP_MESH_PLANNER_H
#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_definitions/mesh_model.h>
#include <cpt_planning/implementation/rmp_mesh_planner.h>
#include <cpt_planning/interface/surface_planner.h>
#include <cpt_planning_eval/chomp/chomp_optimizer.h>

class ChompMeshPlanner : public cad_percept::planning::SurfacePlanner {
 public:
  ChompMeshPlanner(std::string meshpath, chomp::ChompParameters params);

  inline const std::string getName() const { return "CHOMP"; }

  const SurfacePlanner::Result plan(const Eigen::Vector3d start, const Eigen::Vector3d goal,
                                    std::vector<Eigen::Vector3d> *states_out);

  double getMeshDistance(const Eigen::VectorXd& position);

  cad_percept::cgal::MeshModel::Ptr model_;
  chomp::ChompOptimizer chomper_;

};
#endif  // CPT_CHOMP_PLANNING_CHOMP_MESH_PLANNER_H
