#include "plane_extraction/plane_extraction_lib.h"

class PlaneExtractionLib::HoughAccumulator {
 public:
  HoughAccumulator() : bin_values_(new pcl::PointCloud<pcl::PointXYZ>){};

  void vote(Eigen::Vector3d vote, int voter) {
    pcl::PointXYZ vote_point(vote(0), vote(1), vote(2));
    kdtree_.nearestKSearch(vote_point, 1, bin_index_, bin_index_dist_);
    ++bins_[bin_index_[0]];
    // std::cout << "Voted" << std::endl;
    voter_ids_[bin_index_[0]].push_back(voter);
  };
  void findmaxima(double min_vote_threshold, std::vector<double> &plane_coefficients,
                  std::vector<std::vector<int>> &get_voter_ids, int k_of_maxima_suppression) {
    // Non-maximum Suppression
    if (k_of_maxima_suppression != 0) {
      pcl::PointXYZ max_candidate;
      for (int i = 0; i < accumulator_size; ++i) {
        max_candidate.x = bin_values_->points[i].x;
        max_candidate.y = bin_values_->points[i].y;
        max_candidate.z = bin_values_->points[i].z;
        kdtree_.nearestKSearch(max_candidate, k_of_maxima_suppression + 1, bin_index_,
                               bin_index_dist_);
        // Check neighbourhood, do not care about bin_index[0] as it discribes point itself
        for (int j = 1; j < bin_index_.size(); ++j) {
          if (bins_[i] < bins_[bin_index_[j]]) break;  // non local maxima
          if (j == (bin_index_.size() - 1)) {          // local maxima
            for (int n = 1; n < bin_index_.size(); ++n) {
              bins_[i] += bins_[bin_index_[n]];
              bins_[bin_index_[n]] = 0;
            }
          }
        }
      }
    }

    plane_coefficients.clear();
    get_voter_ids.clear();
    for (int i = 0; i < accumulator_size; ++i) {
      if (bins_[i] >= min_vote_threshold) {
        plane_coefficients.push_back(bin_values_->points[i].x);  // rho
        plane_coefficients.push_back(bin_values_->points[i].y);  // theta
        plane_coefficients.push_back(bin_values_->points[i].z);  // psi

        get_voter_ids.push_back(voter_ids_[i]);
      }
    }
  };
  void reset() {
    voter_ids_.clear();
    bins_.clear();
    bins_.resize(accumulator_size);
    voter_ids_.resize(accumulator_size, std::vector<int>(0));
  }

 protected:
  void initAccumulator(pcl::PointCloud<pcl::PointXYZ>::Ptr accumulator_pc) {
    bins_.resize(accumulator_size);
    voter_ids_.resize(accumulator_size, std::vector<int>(0));
    pcl::copyPointCloud(*accumulator_pc, *bin_values_);
    kdtree_.setInputCloud(bin_values_);
  }
  int accumulator_size;

 private:
  std::vector<int> bins_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr bin_values_;

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
  pcl::PointXYZ vote_point_;
  int k_kdtree_ = 1;
  std::vector<int> bin_index_;
  std::vector<float> bin_index_dist_;
  std::vector<std::vector<int>> voter_ids_;
};

class PlaneExtractionLib::ArrayAccumulator : public PlaneExtractionLib::HoughAccumulator {
 public:
  ArrayAccumulator(Eigen::Vector3d bin_minima, Eigen::Vector3d bin_size, Eigen::Vector3d bin_maxima)
      : array_accumulator(new pcl::PointCloud<pcl::PointXYZ>) {
    bin_number_rho_ = (int)((bin_maxima[0] - bin_minima[0]) / bin_size[0] + 1);
    bin_number_theta_ = (int)((bin_maxima[1] - bin_minima[1]) / bin_size[1] + 1);
    bin_number_psi_ = (int)((bin_maxima[2] - bin_minima[2]) / bin_size[2] + 1);
    accumulator_size = bin_number_rho_ * bin_number_theta_ * bin_number_psi_;

    std::cout << "ArrayAccumualtor uses a total of " << accumulator_size << std::endl;
    std::cout << " rho bins: " << bin_number_rho_ << " theta bins: " << bin_number_theta_
              << " psi bins: " << bin_number_psi_ << std::endl;
    // Implementation for array accumualtor, should be able to easily adapt to other designs
    pcl::PointXYZ bin_point;
    for (int d_rho = 0; d_rho < bin_number_rho_; ++d_rho) {
      for (int d_theta = 0; d_theta < bin_number_theta_; ++d_theta) {
        for (int d_psi = 0; d_psi < bin_number_psi_; ++d_psi) {
          bin_point.x = d_rho * bin_size[0] + bin_minima[0];
          bin_point.y = d_theta * bin_size[1] + bin_minima[1];
          bin_point.z = d_psi * bin_size[2] + bin_minima[2];
          array_accumulator->push_back(bin_point);
        }
      }
    }
    this->initAccumulator(array_accumulator);
  }

 private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr array_accumulator;
  int bin_number_rho_;
  int bin_number_theta_;
  int bin_number_psi_;
};

class PlaneExtractionLib::BallAccumulator : public PlaneExtractionLib::HoughAccumulator {
 public:
  BallAccumulator(Eigen::Vector3d bin_minima, Eigen::Vector3d bin_size, Eigen::Vector3d bin_maxima)
      : array_accumulator(new pcl::PointCloud<pcl::PointXYZ>) {
    bin_number_rho_ = (int)((bin_maxima[0] - bin_minima[0]) / bin_size[0] + 1);
    bin_number_theta_ = (int)((bin_maxima[1] - bin_minima[1]) / bin_size[1] + 1);
    bin_number_psi_ = (int)((bin_maxima[2] - bin_minima[2]) / bin_size[2] + 1);
    accumulator_size = bin_number_rho_ * bin_number_theta_ * bin_number_psi_;
    accumulator_size = bin_number_rho_ * bin_number_theta_ * bin_number_psi_;

    std::cout << "BallAccumualtor uses a total of " << accumulator_size << std::endl;
    std::cout << " rho bins: " << bin_number_rho_ << " theta bins: " << bin_number_theta_
              << " psi bins: " << bin_number_psi_ << std::endl;
    // Implementation for array accumualtor, should be able to easily adapt to other designs
    pcl::PointXYZ bin_point;
    for (int d_rho = 0; d_rho < bin_number_rho_; ++d_rho) {
      for (int d_psi = 0; d_psi < bin_number_psi_; ++d_psi) {
        double delta_theta = 2 * M_PI /
                             (((d_psi + 0.5) * bin_size[2] + bin_minima[2]) *
                              bin_number_theta_);  // Simplified formula from paper
        for (int d_theta = 0; d_theta < bin_number_theta_; ++d_theta) {
          bin_point.x = d_rho * bin_size[0] + bin_minima[0];
          bin_point.y = d_theta * delta_theta + bin_minima[1];
          bin_point.z = d_psi * bin_size[2] + bin_minima[2];
          array_accumulator->push_back(bin_point);
        }
      }
    }
    this->initAccumulator(array_accumulator);
  }

 private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr array_accumulator;
  int bin_number_rho_;
  int bin_number_theta_;
  int bin_number_psi_;
};

// Plane Extraction using RHT
std::vector<double> PlaneExtractionLib::rht_plane_extraction(
    const pcl::PointCloud<pcl::PointXYZ> lidar_frame, ros::Publisher &plane_pub,
    std::string tf_map_frame, ros::NodeHandle &nh_private) {
  std::cout << "///////////////////////////////////////////////" << std::endl;
  std::cout << "         RHT Plane Extraction started          " << std::endl;
  std::cout << "///////////////////////////////////////////////" << std::endl;

  int accumulator_choice = nh_private.param<int>("AccumulatorChoice", 1);
  double rho_resolution = nh_private.param<double>("AccumulatorRhoResolution", 100);
  double theta_resolution = nh_private.param<double>("AccumulatorThetaResolution", 20);
  double psi_resolution = nh_private.param<double>("AccumulatorPsiResolution", 10);
  int min_vote_threshold = nh_private.param<int>("AccumulatorThreshold", 10);
  int k_of_maxima_suppression = nh_private.param<int>("AccumulatorKMaxSuppress", 0);
  float slope_threshold = nh_private.param<float>("AccumulatorSlopeThreshold", 0);

  int max_iteration = nh_private.param<int>("RHTMaxIter", 10000);
  double tol_distance_between_points = nh_private.param<double>("RHTTolDist", 2);
  double min_area_spanned = nh_private.param<double>("RHTMinArea", 0.05);
  int num_main_planes = nh_private.param<int>("RHTPlaneNumber", 0);

  pcl::PointXYZ origin(0, 0, 0);
  double max_distance_to_point = 0;
  for (int i = 0; i < lidar_frame.size(); i++) {
    if ((double)pcl::geometry::distance(origin, lidar_frame.points[i]) > max_distance_to_point)
      max_distance_to_point = (double)pcl::geometry::distance(origin, lidar_frame.points[i]);
  }
  std::cout << "Point furthest apart has a distance of " << max_distance_to_point
            << " to the origin" << std::endl;

  PlaneExtractionLib::HoughAccumulator *accumulator;

  if (accumulator_choice == 2) {
    std::cout << "Initialize ball accumulator" << std::endl;
    Eigen::Vector3d min_coord(0, -(double)M_PI, -(double)M_PI / 2);  // rho theta psi
    Eigen::Vector3d max_coord(max_distance_to_point, (double)M_PI, (double)M_PI / 2);
    Eigen::Vector3d bin_size(rho_resolution, theta_resolution, psi_resolution);
    accumulator = new (PlaneExtractionLib::HoughAccumulator)(
        PlaneExtractionLib::BallAccumulator(min_coord, bin_size, max_coord));
  } else {
    std::cout << "Initialize array accumulator" << std::endl;
    Eigen::Vector3d min_coord(0, -(double)M_PI, -(double)M_PI / 2);  // rho theta psi
    Eigen::Vector3d max_coord(max_distance_to_point, (double)M_PI, (double)M_PI / 2);
    Eigen::Vector3d bin_size(rho_resolution, theta_resolution, psi_resolution);
    accumulator = new (PlaneExtractionLib::HoughAccumulator)(
        PlaneExtractionLib::ArrayAccumulator(min_coord, bin_size, max_coord));
  }

  // Perform RHT
  std::vector<pcl::PointXYZ> sampled_points;
  pcl::PointXYZ reference_point[3];
  pcl::PointXYZ vector_on_plane[2];
  pcl::PointXYZ normal_of_plane;
  double norm_of_normal;
  Eigen::Vector3d vote;

  int pointcloud_size = lidar_frame.size();

  std::cout << "Start voting" << std::endl;
  for (int i = 0; i < max_iteration; ++i) {
    // Sample random points
    reference_point[0] = lidar_frame.points[(rand() % (pointcloud_size + 1))];
    reference_point[1] = lidar_frame.points[(rand() % (pointcloud_size + 1))];
    reference_point[2] = lidar_frame.points[(rand() % (pointcloud_size + 1))];
    sampled_points.push_back(reference_point[0]);
    sampled_points.push_back(reference_point[1]);
    sampled_points.push_back(reference_point[2]);

    // Check if points are too close
    if (pcl::geometry::distance(reference_point[0], reference_point[1]) >
        tol_distance_between_points)
      continue;

    if (pcl::geometry::distance(reference_point[1], reference_point[2]) >
        tol_distance_between_points)
      continue;

    if (pcl::geometry::distance(reference_point[0], reference_point[2]) >
        tol_distance_between_points)
      continue;

    vector_on_plane[0] = pcl::PointXYZ(reference_point[1].x - reference_point[0].x,
                                       reference_point[1].y - reference_point[0].y,
                                       reference_point[1].z - reference_point[0].z);
    vector_on_plane[1] = pcl::PointXYZ(reference_point[2].x - reference_point[0].x,
                                       reference_point[2].y - reference_point[0].y,
                                       reference_point[2].z - reference_point[0].z);
    normal_of_plane = pcl::PointXYZ(
        vector_on_plane[1].y * vector_on_plane[0].z - vector_on_plane[1].z * vector_on_plane[0].y,
        vector_on_plane[1].z * vector_on_plane[0].x - vector_on_plane[1].x * vector_on_plane[0].z,
        vector_on_plane[1].x * vector_on_plane[0].y - vector_on_plane[1].y * vector_on_plane[0].x);

    norm_of_normal = (double)pcl::geometry::distance(origin, normal_of_plane);
    if (norm_of_normal < min_area_spanned) continue;

    vote(0) = normal_of_plane.x * reference_point[0].x + normal_of_plane.y * reference_point[0].y +
              normal_of_plane.z * reference_point[0].z;  // rho
    vote(0) = vote(0) / norm_of_normal;
    if (vote(0) < 0) {
      vote(0) = -vote(0);
      normal_of_plane.x = -normal_of_plane.x;
      normal_of_plane.y = -normal_of_plane.y;
      normal_of_plane.z = -normal_of_plane.z;
    }

    // // Check if points are too close
    // if (pcl::geometry::distance(reference_point[0], reference_point[1]) >
    //     tol_distance_between_points + slope_threshold * vote(0))
    //   continue;

    // if (pcl::geometry::distance(reference_point[1], reference_point[2]) >
    //     tol_distance_between_points + slope_threshold * vote(0))
    //   continue;

    // if (pcl::geometry::distance(reference_point[0], reference_point[2]) >
    //     tol_distance_between_points + slope_threshold * vote(0))
    //   continue;

    vote(1) = atan2(normal_of_plane.y, normal_of_plane.x);  // theta
    vote(2) = atan2(normal_of_plane.z,
                    sqrt(pow(normal_of_plane.x, 2) + pow(normal_of_plane.y, 2)));  // psi

    accumulator->vote(vote, i);
  }
  std::cout << "Voting finished" << std::endl;

  std::vector<double> plane_coefficients;
  std::vector<std::vector<int>> maxima_voter_ids;

  accumulator->findmaxima(min_vote_threshold, plane_coefficients, maxima_voter_ids,
                          k_of_maxima_suppression);

  if (num_main_planes != 0 && maxima_voter_ids.size() > num_main_planes) {
    do {
      accumulator->findmaxima(++min_vote_threshold, plane_coefficients, maxima_voter_ids,
                              k_of_maxima_suppression);
    } while (maxima_voter_ids.size() > num_main_planes);
    std::cout << "Used " << min_vote_threshold << " as threshold" << std::endl;
  }

  // Visualization
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>> extracted_planes;
  pcl::PointXYZRGB converter_point;
  pcl::PointCloud<pcl::PointXYZ> used_inliers;
  for (int i = 0; i < (int)(plane_coefficients.size() / 3); ++i) {
    for (int maxima = 0; maxima < maxima_voter_ids[i].size(); maxima++) {
      used_inliers.push_back(sampled_points[maxima_voter_ids[i][maxima] * 3]);
      used_inliers.push_back(sampled_points[maxima_voter_ids[i][maxima] * 3 + 1]);
      used_inliers.push_back(sampled_points[maxima_voter_ids[i][maxima] * 3 + 2]);
    }
    extracted_planes.push_back(*(new pcl::PointCloud<pcl::PointXYZRGB>));
    pcl::copyPointCloud(used_inliers, extracted_planes.back());
    used_inliers.clear();
  }
  visualize_plane(extracted_planes, plane_pub, tf_map_frame);

  for (int i = 0; i < extracted_planes.size(); ++i) {
    std::cout << plane_coefficients[i * 3] << " " << plane_coefficients[i * 3 + 1] << " "
              << plane_coefficients[i * 3 + 2] << " color: " << i % 8 << std::endl;
  }
  return plane_coefficients;
}

std::vector<double> PlaneExtractionLib::iterative_rht_plane_extraction(
    const pcl::PointCloud<pcl::PointXYZ> lidar_frame, ros::Publisher &plane_pub,
    std::string tf_map_frame, ros::NodeHandle &nh_private) {
  std::cout << "///////////////////////////////////////////////" << std::endl;
  std::cout << "    Iterative RHT Plane Extraction started     " << std::endl;
  std::cout << "///////////////////////////////////////////////" << std::endl;

  double rho_resolution = nh_private.param<double>("iterAccumulatorRhoResolution", 100);
  double theta_resolution = nh_private.param<double>("iterAccumulatorThetaResolution", 20);
  double psi_resolution = nh_private.param<double>("iterAccumulatorPsiResolution", 10);
  int k_of_maxima_suppression = nh_private.param<int>("iterAccumulatorKMaxSuppress", 0);
  int min_vote_threshold = 0;

  int iteration_per_plane = nh_private.param<int>("iterRHTIter", 100);
  double tol_distance_between_points = nh_private.param<double>("iterRHTTolDist", 2);
  double min_area_spanned = nh_private.param<double>("iterRHTMinArea", 0.05);
  int num_main_planes = nh_private.param<int>("iterRHTPlaneNumber", 0);

  pcl::PointCloud<pcl::PointXYZ> copy_lidar_frame;
  pcl::copyPointCloud(lidar_frame, copy_lidar_frame);

  pcl::PointXYZ origin(0, 0, 0);
  double max_distance_to_point = 0;
  for (int i = 0; i < lidar_frame.size(); i++) {
    if ((double)pcl::geometry::distance(origin, copy_lidar_frame.points[i]) > max_distance_to_point)
      max_distance_to_point = (double)pcl::geometry::distance(origin, copy_lidar_frame.points[i]);
  }
  std::cout << "Point furthest apart has a distance of " << max_distance_to_point
            << " to the origin" << std::endl;

  std::cout << "Initialize ball accumulator" << std::endl;
  Eigen::Vector3d min_coord(0, -(double)M_PI, -(double)M_PI / 2);  // rho theta psi
  Eigen::Vector3d max_coord(max_distance_to_point, (double)M_PI, (double)M_PI / 2);
  Eigen::Vector3d bin_size(rho_resolution, theta_resolution, psi_resolution);
  PlaneExtractionLib::HoughAccumulator *accumulator;
  accumulator = new (PlaneExtractionLib::HoughAccumulator)(
      PlaneExtractionLib::BallAccumulator(min_coord, bin_size, max_coord));

  // Perform RHT
  std::vector<pcl::PointXYZ> sampled_points;
  pcl::PointXYZ reference_point[3];
  pcl::PointXYZ vector_on_plane[2];
  pcl::PointXYZ normal_of_plane;
  double norm_of_normal;
  Eigen::Vector3d vote;
  pcl::ExtractIndices<pcl::PointXYZ> indices_filter;
  pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud(new pcl::PointCloud<pcl::PointXYZ>());

  int pointcloud_size = copy_lidar_frame.size();
  std::vector<int> reference_point_index;
  std::vector<int> inliers;

  std::vector<double> plane_coefficients;
  std::vector<std::vector<int>> maxima_voter_ids;

  for (int iter = 0; iter < num_main_planes; ++iter) {
    std::cout << "Start voting" << std::endl;
    for (int i = 0; i < iteration_per_plane; ++i) {
      // Sample random points
      reference_point_index.push_back((rand() % (pointcloud_size + 1)));
      reference_point[0] = copy_lidar_frame.points[reference_point_index.back()];
      reference_point_index.push_back((rand() % (pointcloud_size + 1)));
      reference_point[1] = copy_lidar_frame.points[reference_point_index.back()];
      reference_point_index.push_back((rand() % (pointcloud_size + 1)));
      reference_point[2] = copy_lidar_frame.points[reference_point_index.back()];
      sampled_points.push_back(reference_point[0]);
      sampled_points.push_back(reference_point[1]);
      sampled_points.push_back(reference_point[2]);

      // Check if points are too close
      if (pcl::geometry::distance(reference_point[0], reference_point[1]) >
          tol_distance_between_points)
        continue;

      if (pcl::geometry::distance(reference_point[1], reference_point[2]) >
          tol_distance_between_points)
        continue;

      if (pcl::geometry::distance(reference_point[0], reference_point[2]) >
          tol_distance_between_points)
        continue;

      vector_on_plane[0] = pcl::PointXYZ(reference_point[1].x - reference_point[0].x,
                                         reference_point[1].y - reference_point[0].y,
                                         reference_point[1].z - reference_point[0].z);
      vector_on_plane[1] = pcl::PointXYZ(reference_point[2].x - reference_point[0].x,
                                         reference_point[2].y - reference_point[0].y,
                                         reference_point[2].z - reference_point[0].z);
      normal_of_plane = pcl::PointXYZ(
          vector_on_plane[1].y * vector_on_plane[0].z - vector_on_plane[1].z * vector_on_plane[0].y,
          vector_on_plane[1].z * vector_on_plane[0].x - vector_on_plane[1].x * vector_on_plane[0].z,
          vector_on_plane[1].x * vector_on_plane[0].y -
              vector_on_plane[1].y * vector_on_plane[0].x);

      norm_of_normal = (double)pcl::geometry::distance(origin, normal_of_plane);
      if (norm_of_normal < min_area_spanned) continue;

      vote(0) = normal_of_plane.x * reference_point[0].x +
                normal_of_plane.y * reference_point[0].y +
                normal_of_plane.z * reference_point[0].z;  // rho
      vote(0) = vote(0) / norm_of_normal;
      if (vote(0) < 0) {
        vote(0) = -vote(0);
        normal_of_plane.x = -normal_of_plane.x;
        normal_of_plane.y = -normal_of_plane.y;
        normal_of_plane.z = -normal_of_plane.z;
      }

      vote(1) = atan2(normal_of_plane.y, normal_of_plane.x);  // theta
      vote(2) = atan2(normal_of_plane.z,
                      sqrt(pow(normal_of_plane.x, 2) + pow(normal_of_plane.y, 2)));  // psi

      accumulator->vote(vote, i + iter * iteration_per_plane);
    }
    std::cout << "Voting finished" << std::endl;

    std::vector<double> iter_plane_coefficients;
    std::vector<std::vector<int>> iter_maxima_voter_ids;
    do {
      accumulator->findmaxima(++min_vote_threshold, iter_plane_coefficients, iter_maxima_voter_ids,
                              k_of_maxima_suppression);
    } while (iter_maxima_voter_ids.size() > 1);

    if (iter_maxima_voter_ids.size() == 0) {
      accumulator->findmaxima(--min_vote_threshold, iter_plane_coefficients, iter_maxima_voter_ids,
                              k_of_maxima_suppression);
    }

    plane_coefficients.push_back(iter_plane_coefficients[0]);
    plane_coefficients.push_back(iter_plane_coefficients[1]);
    plane_coefficients.push_back(iter_plane_coefficients[2]);

    maxima_voter_ids.push_back(iter_maxima_voter_ids[0]);

    for (auto voter : iter_maxima_voter_ids[0]) {
      inliers.push_back(reference_point_index[voter * 3]);
      inliers.push_back(reference_point_index[voter * 3 + 1]);
      inliers.push_back(reference_point_index[voter * 3 + 2]);
    }

    boost::shared_ptr<std::vector<int>> inliers_ptr = boost::make_shared<std::vector<int>>(inliers);
    pcl::copyPointCloud(copy_lidar_frame, *filter_cloud);
    indices_filter.setInputCloud(filter_cloud);
    indices_filter.setIndices(inliers_ptr);
    indices_filter.setNegative(true);
    indices_filter.filter(*filter_cloud);
    copy_lidar_frame = *filter_cloud;

    std::cout << copy_lidar_frame.size() << std::endl;

    inliers.clear();
    accumulator->reset();
    min_vote_threshold = 0;
  }

  // Visualization
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>> extracted_planes;
  pcl::PointXYZRGB converter_point;
  pcl::PointCloud<pcl::PointXYZ> used_inliers;
  for (int i = 0; i < (int)(plane_coefficients.size() / 3); ++i) {
    for (int maxima = 0; maxima < maxima_voter_ids[i].size(); maxima++) {
      used_inliers.push_back(sampled_points[maxima_voter_ids[i][maxima] * 3]);
      used_inliers.push_back(sampled_points[maxima_voter_ids[i][maxima] * 3 + 1]);
      used_inliers.push_back(sampled_points[maxima_voter_ids[i][maxima] * 3 + 2]);
    }
    extracted_planes.push_back(*(new pcl::PointCloud<pcl::PointXYZRGB>));
    pcl::copyPointCloud(used_inliers, extracted_planes.back());
    used_inliers.clear();
  }
  visualize_plane(extracted_planes, plane_pub, tf_map_frame);

  for (int i = 0; i < extracted_planes.size(); ++i) {
    std::cout << plane_coefficients[i * 3] << " " << plane_coefficients[i * 3 + 1] << " "
              << plane_coefficients[i * 3 + 2] << " color: " << i % 8 << std::endl;
  }
  return plane_coefficients;
}

// Plane Extraction using pcl tutorial (see also planarSegmentationPCL)
void PlaneExtractionLib::pcl_plane_extraction(
    std::vector<pcl::PointCloud<pcl::PointXYZ>> &extracted_planes,
    std::vector<std::vector<double>> &plane_coefficients,
    const pcl::PointCloud<pcl::PointXYZ> lidar_frame, ros::Publisher &plane_pub,
    std::string tf_map_frame, ros::NodeHandle &nh_private) {
  std::cout << "///////////////////////////////////////////////" << std::endl;
  std::cout << "         PCL Plane Extraction started          " << std::endl;
  std::cout << "///////////////////////////////////////////////" << std::endl;

  double distance_threshold = nh_private.param<double>("PCLDistanceThreshold", 0.01);
  int max_number_of_plane = nh_private.param<int>("PCLMaxNumPlane", 12);
  int min_number_of_inlier = nh_private.param<int>("PCLMinInlier", 15);

  // Find plane with PCL
  pcl::PointCloud<pcl::PointXYZ>::Ptr copy_lidar_frame(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(lidar_frame, *copy_lidar_frame);

  std::cout << "Setup Plane Model Extraction" << std::endl;
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(distance_threshold);

  std::cout << "Start to extract planes" << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr extracted_inlier_points(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ExtractIndices<pcl::PointXYZ> indices_filter;
  pcl::PointXYZ normal_of_plane;
  double norm_of_normal;
  std::vector<double> actuel_plane_coefficients{0, 0, 0};

  do {
    seg.setInputCloud(copy_lidar_frame);
    seg.segment(*inliers, *coefficients);

    // Extract inlier of actuel plane
    extracted_inlier_points->clear();
    for (auto indices : inliers->indices)
      extracted_inlier_points->push_back(copy_lidar_frame->points[indices]);

    // Add to return data
    if (extracted_inlier_points->size() > min_number_of_inlier) {
      extracted_planes.push_back(*extracted_inlier_points);

      // Read plane coefficients
      normal_of_plane.x = coefficients->values[0];
      normal_of_plane.y = coefficients->values[1];
      normal_of_plane.z = coefficients->values[2];

      norm_of_normal = normal_of_plane.x * extracted_inlier_points->points[0].x +
                       normal_of_plane.y * extracted_inlier_points->points[0].y +
                       normal_of_plane.z * extracted_inlier_points->points[0].z;
      if (norm_of_normal < 0) norm_of_normal = -norm_of_normal;

      actuel_plane_coefficients[0] = norm_of_normal;                               // rho
      actuel_plane_coefficients[1] = atan2(normal_of_plane.y, normal_of_plane.x);  // theta
      actuel_plane_coefficients[2] = atan2(
          normal_of_plane.z, sqrt(pow(normal_of_plane.x, 2) + pow(normal_of_plane.y, 2)));  // psi
      plane_coefficients.push_back(actuel_plane_coefficients);

      std::cout << "Plane found (nr. " << extracted_planes.size() << ")" << std::endl;

      indices_filter.setInputCloud(copy_lidar_frame);
      indices_filter.setIndices(inliers);
      indices_filter.setNegative(true);
      indices_filter.filter(*copy_lidar_frame);
    }
  } while (extracted_planes.size() < max_number_of_plane &&
           extracted_inlier_points->size() > min_number_of_inlier &&
           copy_lidar_frame->size() > min_number_of_inlier);

  // Visualize plane
  temp_visualize_plane(extracted_planes, plane_pub, tf_map_frame);

  for (int i = 0; i < extracted_planes.size(); ++i) {
    std::cout << plane_coefficients[i][0] << " " << plane_coefficients[i][1] << " "
              << plane_coefficients[i][2] << " color: " << i % 8 << std::endl;
  }
}

// Visualization of planes in rviz
void PlaneExtractionLib::temp_visualize_plane(
    std::vector<pcl::PointCloud<pcl::PointXYZ>> &extracted_planes, ros::Publisher &plane_pub,
    std::string tf_map_frame) {
  std::cout << "Start Visualization" << std::endl;

  int color[8][3] = {{0, 0, 0},     {255, 0, 0},   {0, 255, 0},   {0, 0, 255},
                     {255, 255, 0}, {255, 0, 255}, {0, 255, 255}, {255, 255, 255}};

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_point_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_inlier_points(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  if (extracted_planes.size() == 0) {
    std::cout << "No planes found" << std::endl;
    return;
  } else {
    std::cout << "Found " << extracted_planes.size() << " planes, visualize plane inliers... "
              << std::endl;
    for (std::size_t i = 0; i < extracted_planes.size(); ++i) {
      colored_inlier_points->clear();
      pcl::copyPointCloud(extracted_planes[i], *colored_inlier_points);
      for (std::size_t j = 0; j < colored_inlier_points->size(); ++j) {
        colored_inlier_points->points[j].r = color[i % 8][0];
        colored_inlier_points->points[j].g = color[i % 8][1];
        colored_inlier_points->points[j].b = color[i % 8][2];
        colored_inlier_points->points[j].a = 255;
      }
      *segmented_point_cloud += *colored_inlier_points;
    }
  }
  sensor_msgs::PointCloud2 segmentation_mesg;
  segmented_point_cloud->header.frame_id = tf_map_frame;
  pcl::toROSMsg(*segmented_point_cloud, segmentation_mesg);
  plane_pub.publish(segmentation_mesg);
  std::cout << "Publish plane segmentation" << std::endl;
}

// Visualization of planes in rviz
void PlaneExtractionLib::visualize_plane(
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>> &extracted_planes, ros::Publisher &plane_pub,
    std::string tf_map_frame) {
  std::cout << "Start Visualization" << std::endl;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_point_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  int color[8][3] = {{0, 0, 0},     {255, 0, 0},   {0, 255, 0},   {0, 0, 255},
                     {255, 255, 0}, {255, 0, 255}, {0, 255, 255}, {255, 255, 255}};

  if (extracted_planes.size() == 0) {
    std::cout << "No planes found" << std::endl;
    return;
  } else {
    std::cout << "Found " << extracted_planes.size() << " planes, visualize plane inliers... "
              << std::endl;
    for (std::size_t i = 0; i < extracted_planes.size(); ++i) {
      for (std::size_t j = 0; j < extracted_planes[i].points.size(); ++j) {
        extracted_planes[i].points[j].r = color[i % 8][0];
        extracted_planes[i].points[j].g = color[i % 8][1];
        extracted_planes[i].points[j].b = color[i % 8][2];
        extracted_planes[i].points[j].a = 255;
      }
      *segmented_point_cloud += extracted_planes[i];
    }
  }
  sensor_msgs::PointCloud2 segmentation_mesg;
  segmented_point_cloud->header.frame_id = tf_map_frame;
  pcl::toROSMsg(*segmented_point_cloud, segmentation_mesg);
  plane_pub.publish(segmentation_mesg);
  std::cout << "Publish plane segmentation" << std::endl;
}