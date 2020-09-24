
#ifndef CPT_OMPL_PLANNING_OMPL_MESH_SAMPLING_PLANNER_H
#define CPT_OMPL_PLANNING_OMPL_MESH_SAMPLING_PLANNER_H
#include <CGAL/point_generators_3.h>
#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_definitions/mesh_model.h>
#include <cpt_planning/implementation/rmp_mesh_planner.h>
#include <cpt_planning/interface/surface_planner.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <visualization_msgs/MarkerArray.h>

#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class MeshManifoldSampler : public ob::StateSampler {
 public:
  MeshManifoldSampler(const ob::StateSpace* space, cad_percept::cgal::MeshModel::Ptr model)
      : StateSampler(space), model_(model) {
    model_->getArea();

    rng_mesh_ =
        std::make_shared<CGAL::Random_points_in_triangle_mesh_3<cad_percept::cgal::Polyhedron>>(
            model->getMeshRef());
  }

  void sampleUniform(ob::State* state) override {
    double* val = static_cast<ob::RealVectorStateSpace::StateType*>(state)->values;
    // set val[0],val[1]
    std::vector<cad_percept::cgal::Point> points;

    std::copy_n(*rng_mesh_, 1, std::back_inserter(points));
    rng_mesh_->operator++();

    val[0] = points[0].x();
    val[1] = points[0].y();
    val[2] = points[0].z();
  }
  // We don't need this in the example below.
  void sampleUniformNear(ob::State* /*state*/, const ob::State* /*near*/,
                         const double /*distance*/) override {
    throw ompl::Exception("MyValidStateSampler::sampleNear", "not implemented");
  }

  void sampleGaussian(ob::State* state, const ob::State* mean, double stdDev) override {
    throw ompl::Exception("MyValidStateSampler::sampleGaussian", "not implemented");
  }

 protected:
  cad_percept::cgal::MeshModel::Ptr model_;
  std::shared_ptr<CGAL::Random_points_in_triangle_mesh_3<cad_percept::cgal::Polyhedron>> rng_mesh_;
};

class OMPLMeshSamplingPlanner : cad_percept::planning::SurfacePlanner {
 public:
  OMPLMeshSamplingPlanner(std::string meshpath, bool connect);

  inline ob::StateSamplerPtr allocMeshManifoldSampler(const ob::StateSpace* space) {
    return std::make_shared<MeshManifoldSampler>(space, model_);
  }

  inline bool meshStateValidityChecker(const ob::State* state) {
    const double* val = static_cast<const ob::RealVectorStateSpace::StateType*>(state)->values;

    cad_percept::cgal::Point pt(val[0], val[1], val[2]);

    return (model_->squaredDistance(pt) < 0.01 * 0.01);
  }

  inline const std::string getName() const { return rrt_connect_ ?  "RRTConnect" : "RRTStar"; }

  const SurfacePlanner::Result plan(const Eigen::Vector3d start, const Eigen::Vector3d goal,
                                    std::vector<Eigen::Vector3d>* states_out);

  double solve_time_{1.0};
  bool rrt_connect_{false};
  cad_percept::cgal::MeshModel::Ptr model_;
};

#endif  // CPT_OMPL_PLANNING_OMPL_MESH_SAMPLING_PLANNER_H
