#ifndef CPT_PLANNING_EVAL_OMPL_MESH_SAMPLING_PLANNER_H
#define CPT_PLANNING_EVAL_OMPL_MESH_SAMPLING_PLANNER_H
#include <CGAL/point_generators_3.h>
#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_definitions/mesh_model.h>
#include <cpt_planning/implementation/rmp_mesh_planner.h>
#include <cpt_planning/interface/surface_planner.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/BiTRRT.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <visualization_msgs/MarkerArray.h>

#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace cad_percept {
namespace planning {

/***
 * Constrained sampler that generates uniformly distributed points
 * on the mesh surface.
 */
class MeshManifoldSampler : public ob::StateSampler {
 public:
  MeshManifoldSampler(const ob::StateSpace* space, cad_percept::cgal::MeshModel::Ptr model)
      : StateSampler(space), model_(model) {
    // Create CGAL based sampler (uniform w.r.t surface, normalizes by triangles size).
    rng_mesh_ =
        std::make_shared<CGAL::Random_points_in_triangle_mesh_3<cad_percept::cgal::Polyhedron>>(
            model->getMeshRef());
  }

  void sampleUniform(ob::State* state) override {
    double* val = static_cast<ob::RealVectorStateSpace::StateType*>(state)->values;
    // set val[0],val[1]
    std::vector<cad_percept::cgal::Point> points;

    // get next point from CGAL rng
    std::copy_n(*rng_mesh_, 1, std::back_inserter(points));
    rng_mesh_->operator++();

    val[0] = points[0].x();
    val[1] = points[0].y();
    val[2] = points[0].z();
  }
  // We don't need this in the example below.
  void sampleUniformNear(ob::State* /*state*/, const ob::State* /*near*/,
                         const double /*distance*/) override {
    throw ompl::Exception("MeshManifoldSampler::sampleNear", "not implemented");
  }

  void sampleGaussian(ob::State* state, const ob::State* mean, double stdDev) override {
    throw ompl::Exception("MeshManifoldSampler::sampleGaussian", "not implemented");
  }

 protected:
  cad_percept::cgal::MeshModel::Ptr model_;
  std::shared_ptr<CGAL::Random_points_in_triangle_mesh_3<cad_percept::cgal::Polyhedron>> rng_mesh_;
};

/***
 * RRT Planner that uses constrained sampling on the surface
 * plus an edge validator (that checkes if random samples on surface
 * can be connected without intersecting geometry).
 */
class OMPLMeshSamplingPlanner : public cad_percept::planning::SurfacePlanner {
 public:
  /***
   * Creates a constrained-sampling based RRT planner
   * @param meshpath Path to off file
   * @param connect Just search for fastest connection (RRT Connect instead of RRT*) if true
   * @param time Allocated solution time
   * @param surface_tolerance Tolerance in [m] for state checking
   * @param checking_resolution Resolution for edge validity checking (dimensionless, %)
   */
  OMPLMeshSamplingPlanner(std::string meshpath, bool connect, double time,
                          double surface_tolerance = 0.01, double checking_resolution = 0.03);

  inline ob::StateSamplerPtr allocMeshManifoldSampler(const ob::StateSpace* space) {
    return std::make_shared<MeshManifoldSampler>(space, model_);
  }

  /***
   * Checks if a sampled state is on the surface.
   */
  inline bool meshStateValidityChecker(const ob::State* state) {
    const double* val = static_cast<const ob::RealVectorStateSpace::StateType*>(state)->values;

    cad_percept::cgal::Point pt(val[0], val[1], val[2]);
    return (model_->squaredDistance(pt) < surface_tolerance_squared_);
  }

  inline const std::string getName() const {
    return (rrt_connect_ ? "RRTConnect" : "RRTStar") + std::to_string((int)(solve_time_ * 1000.0));
  }

  const SurfacePlanner::Result plan(const Eigen::Vector3d start, const Eigen::Vector3d goal,
                                    std::vector<Eigen::Vector3d>* states_out);

  SurfacePlanner::EdgeList getEdges() const { return rrt_tree_; }

  // squared tolerance for checking if state
  // is valid (tolerance in surface distance in [m])
  double surface_tolerance_squared_{0.01 * 0.01};
  double checking_resolution_{0.03};
  SurfacePlanner::EdgeList rrt_tree_;
  double solve_time_{1.0};
  bool rrt_connect_{false};
  cad_percept::cgal::MeshModel::Ptr model_;
};
}  // namespace planning
}  // namespace cad_percept
#endif  // CPT_PLANNING_EVAL_OMPL_MESH_SAMPLING_PLANNER_H
