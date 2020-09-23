
#ifndef CPT_OMPL_PLANNING_OMPL_MESH_SAMPLING_PLANNER_H
#define CPT_OMPL_PLANNING_OMPL_MESH_SAMPLING_PLANNER_H
#include <CGAL/point_generators_3.h>
#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_definitions/mesh_model.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ros/ros.h>
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

class OMPLMeshSamplingPlanner {
 public:
  OMPLMeshSamplingPlanner(bool connect) : rrt_connect_(connect) {
    std::cout << cad_percept::cgal::MeshModel::create(
                     "/home/mpantic/ws/rmp/src/manifold_simulations/models/hilo_roof/meshes/"
                     "hilo_reconstructed.off",
                     &model_, true)
              << std::endl;

    ros::NodeHandle node_handle;
  }

  ob::StateSamplerPtr allocMeshManifoldSampler(const ob::StateSpace* space) {
    return std::make_shared<MeshManifoldSampler>(space, model_);
  }

  bool meshStateValidityChecker(const ob::State* state) {
    const double* val = static_cast<const ob::RealVectorStateSpace::StateType*>(state)->values;

    cad_percept::cgal::Point pt(val[0], val[1], val[2]);

    return (model_->squaredDistance(pt) < 0.01 * 0.01);
  }

  bool plan(Eigen::Vector3d startpoint, Eigen::Vector3d endpoint,
            std::vector<Eigen::Vector3d>* pathpoints) {
    std::cout << "Plantest2" << std::endl;
    ob::StateSpacePtr space(new ob::RealVectorStateSpace(3));
    space->as<ob::RealVectorStateSpace>()->setBounds(-23.0, 23.0);

    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));

    si->setStateValidityChecker(
        std::bind(&OMPLMeshSamplingPlanner::meshStateValidityChecker, this, std::placeholders::_1));
    si->setStateValidityCheckingResolution(0.01);  // 1%

    auto ss = std::make_shared<og::SimpleSetup>(si);
    space->setStateSamplerAllocator(
        std::bind(&OMPLMeshSamplingPlanner::allocMeshManifoldSampler, this, std::placeholders::_1));
    // set up solver

    ob::PlannerPtr optimizingPlanner;
    if (rrt_connect_) {
      optimizingPlanner.reset(new og::RRTConnect(si));
    } else {
      optimizingPlanner.reset(new og::RRTstar(si));
    }

    optimizingPlanner->setup();

    ss->setPlanner(optimizingPlanner);

    // set start & goal
    ob::ScopedState<> start(space);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = startpoint.x();
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = startpoint.y();
    start->as<ob::RealVectorStateSpace::StateType>()->values[2] = startpoint.z();

    ob::ScopedState<> goal(space);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = endpoint.x();
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = endpoint.y();
    goal->as<ob::RealVectorStateSpace::StateType>()->values[2] = endpoint.z();

    ss->setStartAndGoalStates(start, goal);

    std::cout << "Starting solve" << std::endl;
    ob::PlannerStatus solved = ss->solve(solve_time_);
     if (solved != ob::PlannerStatus::EXACT_SOLUTION) {
      return false;
    }

    // get path
    ompl::geometric::PathGeometric path = ss->getSolutionPath();

    for (auto state : path.getStates()) {
      Eigen::Vector3d pt;
      pt.x() = state->as<ob::RealVectorStateSpace::StateType>()->values[0];
      pt.y() = state->as<ob::RealVectorStateSpace::StateType>()->values[1];
      pt.z() = state->as<ob::RealVectorStateSpace::StateType>()->values[2];
      pathpoints->push_back(pt);
    }
    return true;
  }

  double solve_time_{1.0};
  bool rrt_connect_{false};
  cad_percept::cgal::MeshModel::Ptr model_;
};

#endif  // CPT_OMPL_PLANNING_OMPL_MESH_SAMPLING_PLANNER_H
