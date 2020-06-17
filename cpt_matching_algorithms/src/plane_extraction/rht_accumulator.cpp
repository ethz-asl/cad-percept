#include "plane_extraction/rht_accumulator.h"

namespace cad_percept {
namespace matching_algorithms {

void HoughAccumulator::vote(Eigen::Vector3d vote, const int (&voter)[3]) {
  bin_index_ = getBinIndexFromVector(vote);
  bin_votes_[bin_index_] += 1;
  voter_ids_[bin_index_].push_back(voter[0]);
  voter_ids_[bin_index_].push_back(voter[1]);
  voter_ids_[bin_index_].push_back(voter[2]);
}

void HoughAccumulator::findMaxima(int min_vote_threshold,
                                  std::vector<Eigen::Vector3d> &plane_coefficients_out,
                                  std::vector<std::vector<int>> &voter_ids_out, int k) {
  plane_coefficients_out.clear();
  voter_ids_out.clear();
  for (int i = 0; i < accumulator_size; ++i) {
    if (bin_votes_(1, i) >= min_vote_threshold) {
      if (k != 0) nonMaximumSuppression(i, k);
      plane_coefficients_out.push_back(getBinValueFromIndex(i));
      voter_ids_out.push_back(voter_ids_[i]);
    }
  }
}

void HoughAccumulator::findMaximumPlane(int num_main_planes,
                                        std::vector<Eigen::Vector3d> &plane_coefficients_out,
                                        std::vector<std::vector<int>> &voter_ids_out, int k) {
  plane_coefficients_out.clear();
  voter_ids_out.clear();
  for (int i = 0; i < num_main_planes; ++i) {
    bin_votes_.maxCoeff(&maximum_idx_);
    if (k != 0) nonMaximumSuppression(maximum_idx_, k);
    plane_coefficients_out.push_back(getBinValueFromIndex(maximum_idx_));
    voter_ids_out.push_back(voter_ids_[maximum_idx_]);
    bin_votes_(1, maximum_idx_) = 0;
  }
}

void HoughAccumulator::reset() {
  bin_votes_ = Eigen::Matrix<int, 1, Eigen::Dynamic>::Zero(1, accumulator_size);
  voter_ids_.clear();
  voter_ids_.resize(accumulator_size, std::vector<int>(0));
}

void HoughAccumulator::normalizeRTP(Eigen::Vector3d &rtp_input) {
  // Correction for psi
  rtp_input[2] = loop_mod(rtp_input[2] + half_pi_, two_pi_);
  if (rtp_input[2] >= M_PI) {
    // if psi lies on the other side rotate theta and correct psi
    rtp_input[2] = two_pi_ - rtp_input[2];
    rtp_input[1] = rtp_input[1] + M_PI;
  }
  rtp_input[1] = loop_mod(rtp_input[1], two_pi_);
  rtp_input[2] = rtp_input[2] - half_pi_;
}

double HoughAccumulator::loop_mod(double x, double y) {
  return std::fmod((std::fmod(x, y) + y), y);
}

void HoughAccumulator::nonMaximumSuppression(int index, int k) {
  neighbor_index_.clear();
  getNeighborIndex(k, index, neighbor_index_);
  for (auto neighor_id : neighbor_index_) {
    if (bin_votes_[index] < bin_votes_[neighor_id]) {
      // No local maxima
      return;
    }
  }
  for (auto neighbor_idx : neighbor_index_) {
    bin_votes_[index] += bin_votes_[neighbor_idx];
    bin_votes_[neighbor_idx] = 0;
    voter_ids_[index].insert(voter_ids_[index].end(), voter_ids_[neighbor_idx].begin(),
                             voter_ids_[neighbor_idx].end());
    voter_ids_[neighbor_idx].clear();
  }
}

ArrayAccumulator::ArrayAccumulator(const Eigen::Vector3d &bin_minima,
                                   const Eigen::Vector3d &bin_size,
                                   const Eigen::Vector3d &bin_maxima) {
  bin_minima_ = bin_minima;
  bin_size_ = bin_size;
  bin_maxima_ = bin_maxima;

  bin_number_rho_ = (int)((bin_maxima[0] - bin_minima[0]) / bin_size[0] + 1);
  bin_number_theta_ = (int)((bin_maxima[1] - bin_minima[1]) / bin_size[1] + 1);
  bin_number_psi_ = (int)((bin_maxima[2] - bin_minima[2]) / bin_size[2] + 1);

  std::cout << "Initialize array accumulator" << std::endl;
  std::cout << "rho bins: " << bin_number_rho_ << " theta bins: " << bin_number_theta_
            << " psi bins: " << bin_number_psi_ << std::endl;

  // // Create tensor
  // uint bin_index = 0;
  // std::vector<accumulatorBin> bin_values_psi;
  // std::vector<std::vector<accumulatorBin>> bin_values_theta_psi;
  // for (int d_rho = 0; d_rho < bin_number_rho_; ++d_rho) {
  //   bin_values_theta_psi.clear();
  //   for (int d_theta = 0; d_theta < bin_number_theta_; ++d_theta) {
  //     bin_values_psi.clear();
  //     for (int d_psi = 0; d_psi < bin_number_psi_; ++d_psi) {
  //       bin_values_psi.push_back({Eigen::Vector3d(d_rho * bin_size[0] + bin_minima[0],
  //                                                 d_theta * bin_size[1] + bin_minima[1],
  //                                                 d_psi * bin_size[2] + bin_minima[2]),
  //                                 bin_index});
  //       index_to_rtp.push_back(Eigen::Vector3i(d_rho, d_theta, d_psi));
  //       bin_index++;
  //     }
  //     bin_values_theta_psi.push_back(bin_values_psi);
  //   }
  //   bin_values.push_back(bin_values_theta_psi);
  // }

  accumulator_size = bin_number_rho_ * bin_number_theta_ * bin_number_psi_;

  this->reset();
}

int ArrayAccumulator::getBinIndexFromVector(Eigen::Vector3d &rtp_input) {
  normalizeRTP(rtp_input);
  rho_index = rtp_input[0] / bin_maxima_[0] * bin_number_rho_;
  theta_index =
      (rtp_input[1] - bin_minima_[1]) / (bin_maxima_[1] - bin_minima_[1]) * bin_number_theta_;
  psi_index = (rtp_input[2] - bin_minima_[2]) / (bin_maxima_[2] - bin_minima_[2]) * bin_number_psi_;
  return rho_index * bin_number_theta_ * bin_number_psi_ + theta_index * bin_number_psi_ +
         psi_index;
}

Eigen::Vector3d ArrayAccumulator::getBinValueFromIndex(int index) {
  rtp_index = getRTPIndexFromBinIndex(index);
  rtp = Eigen::Vector3d((double)rtp_index[0] / bin_number_rho_ * bin_maxima_[0],
                        (double)rtp_index[1] / bin_number_theta_ * 2 * M_PI,
                        (double)rtp_index[2] / bin_number_psi_ * M_PI - M_PI / 2);
  return rtp;
}

Eigen::Vector3i ArrayAccumulator::getRTPIndexFromBinIndex(int index) {
  rho_index = (int)((double)index / (bin_number_theta_ * bin_number_psi_));
  theta_index = (int)(index % (rho_index * bin_number_theta_ * bin_number_psi_) / bin_number_psi_);
  psi_index = index % bin_number_psi_;
  rtp_index = Eigen::Vector3i(rho_index, theta_index, psi_index);
  return rtp_index;
}

bool ArrayAccumulator::getBinIndexFromRTPIndex(int &index, Eigen::Vector3i rtp_input) {
  if (rtp_input[0] < 0 || bin_number_rho_ <= rtp_input[0]) return false;
  index = rtp_input[0] * (bin_number_theta_ * bin_number_psi_) + rtp_input[1] * bin_number_psi_ +
          rtp_input[2];
  return true;
}

void ArrayAccumulator::getNeighborIndex(int neighbor_nr, int index,
                                        std::vector<int> &neighbor_index_out) {
  rtp_index = getRTPIndexFromBinIndex(index);
  for (int d_rho = -neighbor_nr; d_rho < neighbor_nr; ++d_rho) {
    // Skip if rho does not lay in range
    if (!(0 <= rtp_index[0] + d_rho && rtp_index[0] + d_rho < bin_number_rho_)) continue;
    for (int d_theta = -neighbor_nr; d_theta < neighbor_nr; ++d_theta) {
      for (int d_psi = -neighbor_nr; d_psi < neighbor_nr; ++d_psi) {
        // Correct for loops in theta and psi
        psi_index = loop_mod(rtp_index[2] + d_psi, 2 * bin_number_psi_);
        theta_index = d_theta + rtp_index[1];
        if (psi_index >= bin_number_psi_) {
          psi_index = 2 * bin_number_psi_ - psi_index - 1;
          theta_index = theta_index + bin_number_theta_ / 2;
        }
        theta_index = loop_mod(theta_index, bin_number_theta_);
        // Skip if index ends up at same index
        if (rtp_index[0] == rtp_index[0] + d_rho && rtp_index[1] == theta_index &&
            rtp_index[2] == psi_index)
          continue;
        if (getBinIndexFromRTPIndex(neighbor_id,
                                    Eigen::Vector3i(rtp_index[0] + d_rho, theta_index, psi_index)))
          neighbor_index_out.push_back(neighbor_id);
      }
    }
  }
}

BallAccumulator::BallAccumulator(const Eigen::Vector3d &bin_minima, const Eigen::Vector3d &bin_size,
                                 const Eigen::Vector3d &bin_maxima) {
  bin_minima_ = bin_minima;
  bin_size_ = bin_size;
  bin_maxima_ = bin_maxima;

  bin_number_rho_ = (int)((bin_maxima[0] - bin_minima[0]) / bin_size[0] + 1);
  int bin_number_theta = (int)((bin_maxima[1] - bin_minima[1]) / bin_size[1] + 1);
  bin_number_psi_ = (int)((bin_maxima[2] - bin_minima[2]) / bin_size[2] + 1);

  std::cout << "Initialize ball accumulator" << std::endl;

  for (int d_psi = 0; d_psi < bin_number_psi_; ++d_psi) {
    bin_number_theta_.push_back(
        (int)(std::max(1.0, bin_number_theta * std::cos(d_psi * bin_size[2] + bin_minima[2]))));
    bin_tot_number_theta_psi_ += bin_number_theta_.back();
  }

  std::cout << "rho bins: " << bin_number_rho_ << " theta bins: variable (max: " << bin_number_theta
            << ") psi bins: " << bin_number_psi_ << std::endl;

  accumulator_size = bin_number_rho_ * bin_tot_number_theta_psi_;

  this->reset();
}

int BallAccumulator::getBinIndexFromVector(Eigen::Vector3d &rtp_input) {
  normalizeRTP(rtp_input);
  rho_index = rtp_input[0] / bin_maxima_[0] * bin_number_rho_;
  psi_index = (rtp_input[2] - bin_minima_[2]) / (bin_maxima_[2] - bin_minima_[2]) * bin_number_psi_;
  theta_index = (rtp_input[1] - bin_minima_[1]) / (bin_maxima_[1] - bin_minima_[1]) *
                bin_number_theta_[psi_index];
  return rho_index * bin_tot_number_theta_psi_ +
         std::accumulate(bin_number_theta_.begin(), bin_number_theta_.begin() + psi_index, 0) +
         theta_index;
}

Eigen::Vector3d BallAccumulator::getBinValueFromIndex(int index) {
  rtp_index = getRTPIndexFromBinIndex(index);
  rtp = Eigen::Vector3d((double)rtp_index[0] / bin_number_rho_ * bin_maxima_[0],
                        (double)rtp_index[1] / bin_number_theta_[rtp_index[2]] * 2 * M_PI,
                        (double)rtp_index[2] / bin_number_psi_ * M_PI - M_PI / 2);
  return rtp;
}

Eigen::Vector3i BallAccumulator::getRTPIndexFromBinIndex(int index) {
  rho_index = (int)((double)index / bin_tot_number_theta_psi_);
  index -= rho_index * bin_tot_number_theta_psi_;
  int sum_of_theta = 0;
  psi_index = 0;
  for (int d_psi = 0; d_psi < bin_number_psi_; ++d_psi) {
    if (sum_of_theta >= index) break;
    sum_of_theta += bin_number_theta_[psi_index];
    psi_index = d_psi;
  }
  theta_index =
      index - std::accumulate(bin_number_theta_.begin(), bin_number_theta_.begin() + psi_index, 0);
  rtp_index = Eigen::Vector3i(rho_index, theta_index, psi_index);
  return rtp_index;
}

bool BallAccumulator::getBinIndexFromRTPIndex(int &index, Eigen::Vector3i rtp_input) {
  if (rtp_input[0] < 0 || bin_number_rho_ <= rtp_input[0]) return false;
  index = rtp_input[0] * bin_tot_number_theta_psi_ +
          std::accumulate(bin_number_theta_.begin(), bin_number_theta_.begin() + rtp_input[2], 0) +
          rtp_input[1];
  return true;
}

void BallAccumulator::getNeighborIndex(int neighbor_nr, int index,
                                       std::vector<int> &neighbor_index_out) {
  rtp_index = getRTPIndexFromBinIndex(index);
  for (int d_rho = -neighbor_nr; d_rho < neighbor_nr; ++d_rho) {
    // Skip if rho does not lay in range
    if (!(0 <= rtp_index[0] + d_rho && rtp_index[0] + d_rho < bin_number_rho_)) continue;
    for (int d_theta = -neighbor_nr; d_theta < neighbor_nr; ++d_theta) {
      // Correct for loops in theta and psi
      for (int d_psi = -neighbor_nr; d_psi < neighbor_nr; ++d_psi) {
        psi_index = loop_mod(rtp_index[2] + d_psi, 2 * bin_number_psi_);
        theta_index = d_theta + rtp_index[1];
        if (psi_index >= bin_number_psi_) {
          psi_index = 2 * bin_number_psi_ - psi_index - 1;
          theta_index = theta_index + bin_number_theta_[rtp_index[2]] / 2;
        }
        // Consider ratio to get correct theta
        theta_index = (int)((double)theta_index / (double)bin_number_theta_[rtp_index[2]] *
                            (double)bin_number_theta_[psi_index]);
        theta_index = loop_mod(theta_index, bin_number_theta_[psi_index]);
        // Skip if index ends up at same index
        if (rtp_index[0] == rtp_index[0] + d_rho && rtp_index[1] == theta_index &&
            rtp_index[2] == psi_index)
          continue;
        if (getBinIndexFromRTPIndex(
                neighbor_id, Eigen::Vector3i(rtp_index[0] + d_rho, theta_index, psi_index))) {
          neighbor_index_out.push_back(neighbor_id);
        }
      }
    }
  }
}

// BallAccumulator::BallAccumulator(const Eigen::Vector3d &bin_minima, const Eigen::Vector3d
// &bin_size,
//                                  const Eigen::Vector3d &bin_maxima) {
//   bin_minima_ = bin_minima;
//   bin_size_ = bin_size;
//   bin_maxima_ = bin_maxima;

//   bin_number_rho_ = (int)((bin_maxima[0] - bin_minima[0]) / bin_size[0] + 1);
//   int bin_number_theta = (int)((bin_maxima[1] - bin_minima[1]) / bin_size[1] + 1);
//   bin_number_psi_ = (int)((bin_maxima[2] - bin_minima[2]) / bin_size[2] + 1);

//   std::cout << "Initialize ball accumulator" << std::endl;
//   std::cout << "rho bins: " << bin_number_rho_ << " theta bins: varying (max: " <<
//   bin_number_theta
//             << ") psi bins: " << bin_number_psi_ << std::endl;

//   // Create tensor
//   for (int d_psi = 0; d_psi < bin_number_psi_; ++d_psi) {
//     // Find number of theta bins and delta thetas for each psi
//     bin_number_theta_.push_back(
//         (int)(std::max(1.0, bin_number_theta * std::cos(d_psi * bin_size[2] +
//         bin_minima[2]))));
//     bin_delta_theta_.push_back(2 * M_PI / (double)(bin_number_theta_.back()));
//   }
//   int max_theta_number = *std::max_element(bin_number_theta_.begin(), bin_number_theta_.end());

//   // uint bin_index = 0;
//   // std::vector<std::vector<accumulatorBin>> bin_values_psi_theta;
//   // std::vector<accumulatorBin> theta_values;
//   // for (int d_rho = 0; d_rho < bin_number_rho_; ++d_rho) {
//   //   bin_values_psi_theta.clear();
//   //   for (int d_psi = 0; d_psi < bin_number_psi_; ++d_psi) {
//   //     theta_values.clear();
//   //     for (int d_theta = 0; d_theta < bin_number_theta_[d_psi]; ++d_theta) {
//   //       theta_values.push_back({Eigen::Vector3d(d_rho * bin_size[0] + bin_minima[0],
//   //                                               d_theta * bin_delta_theta_[d_psi] +
//   //                                               bin_minima[1], d_psi * bin_size[2] +
//   //                                               bin_minima[2]),
//   //                               bin_index});
//   //       index_to_rtp.push_back(Eigen::Vector3i(d_rho, d_theta, d_psi));
//   //       bin_index++;
//   //     }
//   //     bin_values_psi_theta.push_back(theta_values);
//   //   }
//   //   bin_values.push_back(bin_values_psi_theta);
//   // }

//   accumulator_size = bin_index;
//   this->reset();
// }

// Eigen::Vector3i BallAccumulator::getRTPIndexFromVector(Eigen::Vector3d &rtp) {
//   normalizeRTP(rtp);
//   rtp_index[0] = (int)((rtp[0] - bin_minima_[0]) / bin_size_[0]);
//   rtp_index[2] = (int)((rtp[2] - bin_minima_[2]) / bin_size_[2]);
//   rtp_index[1] = (int)((rtp[1] - bin_minima_[1]) / bin_delta_theta_[rtp_index[2]]);
//   return rtp_index;
// }

// Eigen::Vector3i HoughAccumulator::getRTPFromIndex(int bin_index) { return
// index_to_rtp[bin_index]; }

// void BallAccumulator::getNeighborIndex(int neighbor_nr, int index,
//                                        std::vector<int> &neighbor_index_out) {
//   rtp_index = getRTPFromIndex(index);
//   for (int d_rho = -neighbor_nr; d_rho < neighbor_nr; ++d_rho) {
//     // Skip if rho does not lay in range
//     if (!(0 <= rtp_index[0] + d_rho && rtp_index[0] + d_rho < bin_number_rho_)) continue;
//     for (int d_theta = -neighbor_nr; d_theta < neighbor_nr; ++d_theta) {
//       // Correct for loops in theta and psi
//       for (int d_psi = -neighbor_nr; d_psi < neighbor_nr; ++d_psi) {
//         psi_index = loop_mod(rtp_index[2] + d_psi, 2 * bin_number_psi_);
//         theta_index = d_theta + rtp_index[1];
//         if (psi_index >= bin_number_psi_) {
//           psi_index = 2 * bin_number_psi_ - psi_index - 1;
//           theta_index = theta_index + bin_number_theta_[rtp_index[2]] / 2;
//         }
//         // Consider ratio to get correct theta
//         theta_index = (int)((double)theta_index / (double)bin_number_theta_[rtp_index[2]] *
//                             (double)bin_number_theta_[psi_index]);
//         theta_index = loop_mod(theta_index, bin_number_theta_[psi_index]);
//         // Skip if index ends up at same index
//         if (rtp_index[0] == rtp_index[0] + d_rho && rtp_index[1] == theta_index &&
//             rtp_index[2] == psi_index)
//           continue;
//         if (getIndexFromRTP(neighbor_id,
//                             Eigen::Vector3i(rtp_index[0] + d_rho, theta_index, psi_index))) {
//           neighbor_index_out.push_back(neighbor_id);
//         }
//       }
//     }
//   }
// }

// bool BallAccumulator::getValue(accumulatorBin &output, Eigen::Vector3i rtp_idx) {
//   // Change theta and psi value due to construction
//   temp_switch_index_ = rtp_idx[2];
//   rtp_idx[2] = rtp_idx[1];
//   rtp_idx[1] = temp_switch_index_;

//   output = ;  // TODO;
//   return true;
// }

// int BallAccumulator::getBinIndexFromVector(Eigen::Vector3d &rtp) {
//   // TODO
//   rtp_index = getRTPIndexFromVector(rtp);
//   getIndexFromRTP(bin_index_, rtp_index);
//   return bin_index_;
// }

}  // namespace matching_algorithms
}  // namespace cad_percept