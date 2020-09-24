#ifndef CPT_PLANNING_INTERFACE_SURFACE_PLANNER_H_
#define CPT_PLANNING_INTERFACE_SURFACE_PLANNER_H_
#include <Eigen/Dense>
#include <chrono>
#include <iostream>

namespace cad_percept {
namespace planning {

class SurfacePlanner {
 public:
  typedef struct {
    bool success;
    std::chrono::steady_clock::duration duration;
  } Result;

  virtual const Result plan(const Eigen::Vector3d start, const Eigen::Vector3d goal,
                            std::vector<Eigen::Vector3d>* states_out) = 0;
  virtual const std::string getName() const = 0;
};

}  // namespace planning
}  // namespace cad_percept
#endif //CPT_PLANNING_INTERFACE_SURFACE_PLANNER_H_
