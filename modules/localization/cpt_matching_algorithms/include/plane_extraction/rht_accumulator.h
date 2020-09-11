#ifndef RHT_ACCUMULATOR_H_
#define RHT_ACCUMULATOR_H_

#include <Eigen/Dense>
#include <iostream>
#include <vector>

namespace cad_percept {
namespace matching_algorithms {

struct accumulatorBin {
  Eigen::Vector3d bin_value;
  int index;
};

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
  int psi_index;
  int theta_index;
  int neighbor_id;

  uint accumulator_size;
  std::vector<Eigen::Vector3i> index_to_rtp;
  std::vector<std::vector<std::vector<accumulatorBin>>> bin_values;

  // Returns index value according to continuous value
  int getBinIndexFromVector(Eigen::Vector3d &rtp);
  // Returns index according to RTP
  bool getIndexFromRTP(int &index_out, Eigen::Vector3i rtp_idx);
  // Returns RTP index according to index
  Eigen::Vector3i getRTPFromIndex(int bin_index);
  // Returns continuous rho, theta and psi values of bin with index index
  Eigen::Vector3d getValueFromIndex(int index);
  // Returns discretized values of continuous rho, theta and psi values in rtp
  virtual Eigen::Vector3i getRTPIndexFromVector(Eigen::Vector3d &rtp) = 0;
  // Returns list of indexes corresponding to neighbors
  virtual void getNeighborIndex(int neighbor_nr, int index,
                                std::vector<int> &neighbor_index_out) = 0;
  // Returns value in bin value list with rtp index, returns false id rtp index is invalid
  virtual bool getValue(accumulatorBin &output, Eigen::Vector3i rtp_idx) = 0;

  // Helper functions
  // Returns true if index is valid
  bool checkIndex(const Eigen::Vector3i &idx);
  // Normalize rho, theta and psi
  void normalizeRTP(Eigen::Vector3d &rtp);
  // Returns modulo value in a loop manner
  double loop_mod(double x, double y);

 private:
  int bin_index_;
  accumulatorBin bin_value_;
  int maximum_idx_;
  double half_pi_ = M_PI / 2;
  double two_pi_ = 2 * M_PI;

  std::vector<int> neighbor_index_;
  std::vector<std::vector<int>> voter_ids_;
  Eigen::Matrix<int, 1, Eigen::Dynamic> bin_votes_;

  // Applies Non-maximum Suppression on all bins which are at most k bins away (according to max
  // norm) from index
  void nonMaximumSuppression(int index, int k);
};

class ArrayAccumulator : public HoughAccumulator {
 public:
  ArrayAccumulator(const Eigen::Vector3d &bin_minima, const Eigen::Vector3d &bin_size,
                   const Eigen::Vector3d &bin_maxima);
  // Implementation of getRTPIndexFromVector of array accumulator
  virtual Eigen::Vector3i getRTPIndexFromVector(Eigen::Vector3d &rtp);
  // Implementation of getNeighborIndex of array accumulator
  virtual void getNeighborIndex(int neighbor_nr, int index, std::vector<int> &neighbor_index);
  // Implementation of getValue of array accumulator
  virtual bool getValue(accumulatorBin &output, Eigen::Vector3i rtp_idx);

 private:
  Eigen::Vector3d bin_minima_;
  Eigen::Vector3d bin_size_;
  Eigen::Vector3d bin_maxima_;
  int bin_number_rho_;
  int bin_number_theta_;
  int bin_number_psi_;
};

class BallAccumulator : public HoughAccumulator {
 public:
  BallAccumulator(const Eigen::Vector3d &bin_minima, const Eigen::Vector3d &bin_size,
                  const Eigen::Vector3d &bin_maxima);
  // Implementation of getRTPIndexFromVector of array accumulator
  virtual Eigen::Vector3i getRTPIndexFromVector(Eigen::Vector3d &rtp);
  // Implementation of getNeighborIndex of array accumulator
  virtual void getNeighborIndex(int neighbor_nr, int index, std::vector<int> &neighbor_index);
  // Implementation of getValue of array accumulator
  virtual bool getValue(accumulatorBin &output, Eigen::Vector3i rtp_idx);

 private:
  int temp_switch_index_;

  Eigen::Vector3d bin_minima_;
  Eigen::Vector3d bin_size_;
  Eigen::Vector3d bin_maxima_;
  int bin_number_rho_;
  std::vector<int> bin_number_theta_;
  std::vector<double> bin_delta_theta_;
  int bin_number_psi_;
};

}  // namespace matching_algorithms
}  // namespace cad_percept

#endif