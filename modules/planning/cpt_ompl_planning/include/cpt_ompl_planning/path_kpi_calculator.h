#ifndef CPT_OMPL_PLANNING_PATH_KPI_CALCULATOR_H
#define CPT_OMPL_PLANNING_PATH_KPI_CALCULATOR_H
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

    friend std::ostream& operator<<(std::ostream& os, QualityIndicator const& q) {
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
    }
  };

  typedef std::vector<Eigen::Vector3d> PathArray;

  void getSmoothness(PathArray path, Statistic* smoothness_out) {
    Accumulator acc;

    // Idea:
    //  Calculate angular similiraty between subsequent segments
    //  and keep a statistic thereof.

    for (size_t i = 1; i < path.size() - 1; i++) {
      Eigen::Vector3d last_segment = path.at(i - 1) - path.at(i);
      Eigen::Vector3d segment = path.at(i) - path.at(i + 1);
      if (last_segment.norm() < 1e-10 || segment.norm() < 1e-10) {
        continue;  // ignore too small segments
      }

      double cosine_similarity = last_segment.dot(segment) / (last_segment.norm() * segment.norm());
      cosine_similarity =
          std::min(cosine_similarity, 1.0);  // clamp to 1.0 in case of float inaccuracies
      cosine_similarity = std::max(cosine_similarity, -1.0);
      double angular_similarity = 1 - (acos(cosine_similarity) / M_PI);

      if (std::isnan(cosine_similarity)) {
        std::cout << "NAN" << std::endl;
        std::cout << cosine_similarity << std::endl;
        std::cout << angular_similarity << std::endl;
        std::cout << last_segment << std::endl;
        std::cout << segment << std::endl;
      }

      acc(angular_similarity);
    }
    smoothness_out->fromAccumulator(acc);
  }

  void getSurfaceDistance(PathArray path, Statistic* surface_dist_out) {
    Accumulator acc;

    for (size_t i = 1; i < path.size(); i++) {
      Eigen::Vector3d segment = path.at(i) - path.at(i - 1);
      double segment_length = segment.norm();
      // we check at least one point per segment (starting point)
      size_t increments = std::max((int)floor(segment_length / quality_sampling_dist_), 1);

      if (std::abs(increments * quality_sampling_dist_ - segment_length) < 1e-5) {
        // in case the segment is an even divisor, we substract one increment, as we do not want
        // to test a point twice (this end point is the next start point)
        --increments;
      }

      // go along segment in defined increments
      Eigen::Vector3d sampling_dist_along_segment = segment.normalized() * quality_sampling_dist_;
      for (size_t j = 0; j < increments; j++) {
        Eigen::Vector3d position = path.at(i - 1) + sampling_dist_along_segment * j;

        // query distance to mesh ..
        // ( query)
        cad_percept::cgal::Point pt(position.x(), position.y(), position.z());
        double dist = sqrt(model_->squaredDistance(pt));
        acc(dist);
      }
    }
    surface_dist_out->fromAccumulator(acc);
  }

  void getLength(PathArray path, double* length_out, Statistic* segment_length_out) {
    accumulator_set<double, stats<tag::mean, tag::sum, tag::variance, tag::min, tag::max>> acc;

    for (size_t i = 1; i < path.size(); i++) {
      double segment_length = (path.at(i - 1) - path.at(i)).norm();
      acc(segment_length);
    }

    *length_out = boost::accumulators::sum(acc);
    segment_length_out->min = boost::accumulators::min(acc);
    segment_length_out->max = boost::accumulators::max(acc);
    segment_length_out->avg = boost::accumulators::mean(acc);
    segment_length_out->cov = boost::accumulators::variance(acc);
  }

  QualityIndicator calculate(PathArray path) {
    QualityIndicator results;
    if (path.size() < 3) {
      return results;
    }

    results.segments = path.size() - 1;
    getLength(path, &results.length, &results.segment_length);
    if (model_) {
      getSurfaceDistance(path, &results.surface_dist);
    }
    getSmoothness(path, &results.smoothness);
    results.valid = true;
    return results;
  }

  void setModel(cad_percept::cgal::MeshModel::Ptr model) { model_ = model; }

  cad_percept::cgal::MeshModel::Ptr model_{nullptr};
  double quality_sampling_dist_{0.01};  // default 1 cm
};

#endif  // CPT_OMPL_PLANNING_PATH_KPI_CALCULATOR_H
