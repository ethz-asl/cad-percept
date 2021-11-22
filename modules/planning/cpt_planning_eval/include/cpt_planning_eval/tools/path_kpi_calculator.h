#ifndef CPT_PLANNING_EVAL_PATH_KPI_CALCULATOR_H
#define CPT_PLANNING_EVAL_PATH_KPI_CALCULATOR_H
#include <cgal_definitions/mesh_model.h>
#include <cpt_planning/interface/surface_planner.h>
#include <math.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <vector>
using namespace boost::accumulators;

namespace cad_percept {
namespace planning {

class PathKPICalcuator {
  typedef accumulator_set<double, stats<tag::mean, tag::sum, tag::variance, tag::min, tag::max>>
      Accumulator;

 public:
  typedef struct {
    double min{0};
    double max{0};
    double avg{0};
    double cov{0};

    void fromAccumulator(const Accumulator& acc) {
      min = boost::accumulators::min(acc);
      max = boost::accumulators::max(acc);
      avg = boost::accumulators::mean(acc);
      cov = boost::accumulators::variance(acc);
    }
  } Statistic;

  class QualityIndicator {
   public:
    bool valid{false};
    double length{0};
    size_t segments{0};
    Statistic smoothness;
    Statistic segment_length;
    Statistic surface_dist;

    friend std::ostream& operator<<(std::ostream& os, QualityIndicator const& q){
      os << std::setprecision(5) << std::fixed;
      os << "valid\t" << q.valid << std::endl;
      os << "length\t" << q.length << std::endl;
      os << "numsegments\t" << q.segments << std::endl;

      os << "\t\tSegments\tSurfDist\tSmoothness" << std::endl;
      os << "min\t\t" << q.segment_length.min << "\t\t" << q.surface_dist.min << "\t\t"
         << q.smoothness.min << std::endl;
      os << "max\t\t" << q.segment_length.max << "\t\t" << q.surface_dist.max << "\t\t"
         << q.smoothness.max << std::endl;
      os << "avg\t\t" << q.segment_length.avg << "\t\t" << q.surface_dist.avg << "\t\t"
         << q.smoothness.avg << std::endl;
      os << "std\t\t" << sqrt(q.segment_length.cov) << "\t\t" << sqrt(q.surface_dist.cov) << "\t\t"
         << sqrt(q.smoothness.cov) << std::endl;
      return os;
    }
  };


  typedef std::vector<Eigen::Vector3d> PathArray;

  void getSmoothness(PathArray path, Statistic* smoothness_out);

  void getSurfaceDistance(PathArray path, Statistic* surface_dist_out);

  void getLength(PathArray path, double* length_out, Statistic* segment_length_out);

  QualityIndicator calculate(PathArray path);

  void setModel(cad_percept::cgal::MeshModel::Ptr model) { model_ = model; }

  cad_percept::cgal::MeshModel::Ptr model_{nullptr};
  double quality_sampling_dist_{0.01};  // default 1 cm
};
}  // namespace planning
}  // namespace cad_percept

#endif  // CPT_PLANNING_EVAL_PATH_KPI_CALCULATOR_H
