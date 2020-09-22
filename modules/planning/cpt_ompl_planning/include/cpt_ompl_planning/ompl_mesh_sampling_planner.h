
#ifndef CPT_OMPL_PLANNING_OMPL_MESH_SAMPLING_PLANNER_H
#define CPT_OMPL_PLANNING_OMPL_MESH_SAMPLING_PLANNER_H
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// This is a problem-specific sampler that automatically generates valid
// states; it doesn't need to call SpaceInformation::isValid. This is an
// example of constrained sampling. If you can explicitly describe the set valid
// states and can draw samples from it, then this is typically much more
// efficient than generating random samples from the entire state space and
// checking for validity.

class MeshManifoldSampler : public ob::ValidStateSampler {
 public:
  MeshManifoldSampler(const ob::SpaceInformation* si) : ValidStateSampler(si) {
    name_ = "my sampler";
  }
  // Generate a sample in the valid part of the R^3 state space
  // Valid states satisfy the following constraints:
  // -1<= x,y,z <=1
  // if .25 <= z <= .5, then |x| >.8 and |y| >.8
  bool sample(ob::State* state) override {
    double* val = static_cast<ob::RealVectorStateSpace::StateType*>(state)->values;
    double z = rng_.uniformReal(-1, 1);

    if (z > .25 && z < .5) {
      double x = rng_.uniformReal(0, 1.8), y = rng_.uniformReal(0, .2);
      switch (rng_.uniformInt(0, 3)) {
        case 0:
          val[0] = x - 1;
          val[1] = y - 1;
          break;
        case 1:
          val[0] = x - .8;
          val[1] = y + .8;
          break;
        case 2:
          val[0] = y - 1;
          val[1] = x - 1;
          break;
        case 3:
          val[0] = y + .8;
          val[1] = x - .8;
          break;
      }
    } else {
      val[0] = rng_.uniformReal(-1, 1);
      val[1] = rng_.uniformReal(-1, 1);
    }
    val[2] = z;
    assert(si_->isValid(state));
    return true;
  }
  // We don't need this in the example below.
  bool sampleNear(ob::State* /*state*/, const ob::State* /*near*/,
                  const double /*distance*/) override {
    throw ompl::Exception("MyValidStateSampler::sampleNear", "not implemented");
    return false;
  }

 protected:
  ompl::RNG rng_;
};

class OMPLMeshSamplingPlanner {
 public:
  static ob::ValidStateSamplerPtr allocMeshManifoldSampler(const ob::SpaceInformation* si) {
    return std::make_shared<MeshManifoldSampler>(si);
  }

  void plan() {
    std::cout << "Plantest2" << std::endl;
    ob::StateSpacePtr space(new ob::RealVectorStateSpace(2));
    space->as<ob::RealVectorStateSpace>()->setBounds(0.0, 1.0);

    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
    auto ss = std::make_shared<og::SimpleSetup>(si);

    ob::PlannerPtr optimizingPlanner(new og::RRTstar(si));

    // set up solver
    ss->setPlanner(optimizingPlanner);
    ss->getSpaceInformation()->setValidStateSamplerAllocator(this->allocMeshManifoldSampler);

    // set start & goal
    ob::ScopedState<> start(space);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = 0.0;
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = 0.0;

    ob::ScopedState<> goal(space);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = 1.0;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = 1.0;

    ss->setStartAndGoalStates(start, goal);
    ob::PlannerStatus solved = ss->solve(1.0);
    std::cout << solved << std::endl;

    // get path
    ompl::geometric::PathGeometric path = ss->getSolutionPath();
    std::cout << "Length: " << path.length() << std::endl;
    std::cout << "Elems: " << path.getStateCount() << std::endl;

    for (auto state : path.getStates()) {
      std::cout << state->as<ob::RealVectorStateSpace::StateType>()->values[0] << " "
                << state->as<ob::RealVectorStateSpace::StateType>()->values[0] << std::endl;
    }
  }
};

#endif  // CPT_OMPL_PLANNING_OMPL_MESH_SAMPLING_PLANNER_H
