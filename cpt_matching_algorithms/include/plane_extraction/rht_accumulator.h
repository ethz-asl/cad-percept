#ifndef RHT_ACCUMULATOR_H_
#define RHT_ACCUMULATOR_H_

#include <Eigen/Dense>
#include <iostream>
#include <numeric>
#include <vector>

namespace cad_percept {
namespace matching_algorithms {

class HoughAccumulator {
 public:
  HoughAccumulator(){};
  // Adds one vote to the bin corresponding to vote, indexes in voter are added to inliers
  void vote(Eigen::Vector3d vote, const int (&voter)[3]);
  // Find all planes with more votes than min_vote_threshold
  void findMaxima(int min_vote_threshold, std::vector<Eigen::Vector3d> &plane_coefficients_out,
                  std::vector<std::vector<int>> &voter_ids_out, int k);
  // Find the num_main_planes planes with largest vote number, apply non-maximum-suppression on
  // maxima
  void findMaximumPlane(int num_main_planes, std::vector<Eigen::Vector3d> &plane_coefficients_out,
                        std::vector<std::vector<int>> &voter_ids_out, int k);
  // Set all votes back to zero, empty all inliers of each bin
  void reset();

 protected:
  Eigen::Vector3i rtp_index;
  Eigen::Vector3d rtp;
  int rho_index;
  int psi_index;
  int theta_index;
  int neighbor_id;
  int bin_index;

  uint accumulator_size;
  double half_pi = M_PI / 2;
  double two_pi = 2 * M_PI;
  double rho_max;

  // Returns index value according to continuous value
  virtual int getBinIndexFromVector(Eigen::Vector3d &rtp_input) = 0;
  // Returns value of bin (rho, theta, psi) with bin index index
  virtual Eigen::Vector3d getBinValueFromIndex(int index) = 0;
  // Return discretized rho, theta and psi value corresponding to bin index index
  virtual Eigen::Vector3i getRTPIndexFromBinIndex(int index) = 0;

  // Returns list of indexes corresponding to neighbors
  virtual void getNeighborIndex(int neighbor_nr, int index,
                                std::vector<int> &neighbor_index_out) = 0;

  // Helper functions
  // Normalize rho, theta and psi
  void normalizeRTP(Eigen::Vector3d &rtp_input);
  // Returns modulo value in a loop manner
  double loop_mod(double x, double y);

 private:
  int bin_index_;
  int maximum_idx_;

  std::vector<int> neighbor_index_;
  std::vector<std::vector<int>> voter_ids_;
  Eigen::Matrix<int, 1, Eigen::Dynamic> bin_votes_;

  // Applies Non-maximum Suppression on all bins which are at most k bins away (according to max
  // norm) from index
  void nonMaximumSuppression(int index, int k);
};

class ArrayAccumulator : public HoughAccumulator {
 public:
  ArrayAccumulator(const double max_range_scan, const Eigen::Vector3d &bin_size);
  // Implementation of getBinIndexFromVector of array accumulator
  virtual int getBinIndexFromVector(Eigen::Vector3d &rtp_input);
  // Implementation of getRTPIndexFromVector of array accumulator
  virtual Eigen::Vector3d getBinValueFromIndex(int index);
  // Implementation of getNeighborIndex of array accumulator
  virtual Eigen::Vector3i getRTPIndexFromBinIndex(int index);
  // Implementation of getNeighborIndex of array accumulator
  virtual void getNeighborIndex(int neighbor_nr, int index, std::vector<int> &neighbor_index);

 private:
  bool getBinIndexFromRTPIndex(int &index, Eigen::Vector3i rtp_input);

  int bin_number_rho_;
  int bin_number_theta_;
  int bin_number_psi_;
};

class BallAccumulator : public HoughAccumulator {
 public:
  BallAccumulator(const double max_range_scan, const Eigen::Vector3d &bin_size);
  // Implementation of getBinIndexFromVector of array accumulator
  virtual int getBinIndexFromVector(Eigen::Vector3d &rtp_input);
  // Implementation of getRTPIndexFromVector of array accumulator
  virtual Eigen::Vector3d getBinValueFromIndex(int index);
  // Implementation of getNeighborIndex of array accumulator
  virtual Eigen::Vector3i getRTPIndexFromBinIndex(int index);
  // Implementation of getNeighborIndex of array accumulator
  virtual void getNeighborIndex(int neighbor_nr, int index, std::vector<int> &neighbor_index);

 private:
  bool getBinIndexFromRTPIndex(int &index, Eigen::Vector3i rtp_input);

  int bin_number_rho_;
  int bin_tot_number_theta_psi_;
  std::vector<int> bin_number_theta_;
  int bin_number_psi_;
};

}  // namespace matching_algorithms
}  // namespace cad_percept

#endif