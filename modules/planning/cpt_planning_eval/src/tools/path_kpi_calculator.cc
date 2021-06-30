#include <cpt_planning_eval/tools/path_kpi_calculator.h>

namespace cad_percept {
namespace planning {

void PathKPICalcuator::getSmoothness(PathArray path, Statistic* smoothness_out) {
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

void PathKPICalcuator::getSurfaceDistance(PathArray path, Statistic* surface_dist_out) {
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

void PathKPICalcuator::getLength(PathArray path, double* length_out,
                                 Statistic* segment_length_out) {
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

PathKPICalcuator::QualityIndicator PathKPICalcuator::calculate(PathArray path) {
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

}  // namespace planning
}  // namespace cad_percept