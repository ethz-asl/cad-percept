#include "plane_matching/plane_matching_lib.h"

void PlaneMatchLib::prrus(float (&transformTR)[7],
                          const pcl::PointCloud<pcl::PointNormal> scan_planes,
                          const pcl::PointCloud<pcl::PointNormal> map_planes,
                          ros::NodeHandle &nh_private) {
  std::cout << "////  PRRUS Matching Started  ////" << std::endl;
  std::cout << "Number of planes in map: " << map_planes.size() << std::endl;
  std::cout << "Number of planes in scan: " << scan_planes.size() << std::endl;

  int k_for_num_of_map = nh_private.param<int>("PRRUSkMap", scan_planes.size());
  float drop_error_threshold = nh_private.param<float>("PRRUSDropThreshold", 1);
  int max_drop_costs = nh_private.param<int>("PRRUSMaxDropCosts", 1);

  float match_score = 1;
  std::vector<int> plane_assignment(scan_planes.size(), 0);

  // Get number of possible conditions under nearest neighbor
  int total_comb_per_plane = 1;
  for (int i = 1; i < scan_planes.size(); ++i) {
    total_comb_per_plane = total_comb_per_plane * k_for_num_of_map;
  }

  float candidate_score = 0;
  std::vector<int> candidate_plane_assignment(scan_planes.size(), 0);
  int assignment_offset = 1;
  int corresp_map_plane;
  int corresp_map_pair_plane;
  int dropped_costs = 0;
  float cost_of_plane_pair = 0;

  std::vector<int> available_map_planes(k_for_num_of_map);
  std::vector<float> nearest_distances(k_for_num_of_map);
  pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
  pcl::PointCloud<pcl::PointNormal>::Ptr inputcloud(new pcl::PointCloud<pcl::PointNormal>);
  *inputcloud = map_planes;
  kdtree.setInputCloud(inputcloud);

  for (int map_plane_nr = 0; map_plane_nr < map_planes.size(); ++map_plane_nr) {
    std::cout << "Set strongest plane from scan to plane " << map_plane_nr
              << " from map, calculate all possible combinations..." << std::endl;
    candidate_plane_assignment[0] = map_plane_nr;
    kdtree.nearestKSearch(map_planes[map_plane_nr], k_for_num_of_map, available_map_planes,
                          nearest_distances);
    for (int candidate_nr = 0; candidate_nr < total_comb_per_plane; ++candidate_nr) {
      // Create plane assignment
      assignment_offset = 1;
      for (int scan_plane_nr = 1; scan_plane_nr < scan_planes.size(); ++scan_plane_nr) {
        candidate_plane_assignment[scan_plane_nr] =
            available_map_planes[candidate_nr / assignment_offset % available_map_planes.size()];
        assignment_offset = assignment_offset * available_map_planes.size();
      }

      // Evaluate assignment
      candidate_score = 0;
      dropped_costs = 0;
      for (int plane_nr = 0; plane_nr < scan_planes.size(); ++plane_nr) {
        for (int pair_plane_nr = 0; pair_plane_nr < scan_planes.size(); ++pair_plane_nr) {
          corresp_map_plane = candidate_plane_assignment[plane_nr];
          corresp_map_pair_plane = candidate_plane_assignment[pair_plane_nr];
          cost_of_plane_pair = std::abs(
              map_planes.points[corresp_map_plane].normal_x *
                  map_planes.points[corresp_map_pair_plane].normal_x +
              map_planes.points[corresp_map_plane].normal_y *
                  map_planes.points[corresp_map_pair_plane].normal_y +
              map_planes.points[corresp_map_plane].normal_z *
                  map_planes.points[corresp_map_pair_plane].normal_z -
              scan_planes.points[plane_nr].normal_x * scan_planes.points[pair_plane_nr].normal_x -
              scan_planes.points[plane_nr].normal_y * scan_planes.points[pair_plane_nr].normal_y -
              scan_planes.points[plane_nr].normal_z * scan_planes.points[pair_plane_nr].normal_z);

          if (cost_of_plane_pair > drop_error_threshold && dropped_costs < max_drop_costs) {
            ++dropped_costs;
            continue;
          }
          candidate_score = candidate_score - cost_of_plane_pair;
        }
      }

      if (candidate_score > match_score || match_score > 0) {
        match_score = candidate_score;
        plane_assignment = candidate_plane_assignment;
        std::cout << "Found new best candidate with score " << match_score << std::endl;
      }
    }
  }

  for (int plane_nr = 0; plane_nr < scan_planes.size(); ++plane_nr) {
    std::cout << "Plane " << plane_nr << " from Scan matched with " << plane_assignment[plane_nr]
              << " from mesh" << std::endl;
  }

  transform_average(transformTR, plane_assignment, scan_planes, map_planes);
}

void PlaneMatchLib::PlaneDescriptor(float (&transformTR)[7],
                                    const pcl::PointCloud<pcl::PointNormal> scan_planes,
                                    const pcl::PointCloud<pcl::PointNormal> map_planes,
                                    ros::NodeHandle &nh_private) {
  float threshold_horizontal = 0.8;
  float parallel_threshold = 0.5;
  std::cout << "////  Plane Descriptor Started  ////" << std::endl;

  // Seperate planes into vertical and horizontal
  // Use of IMU?
  std::vector<int> map_horizontal_planes;
  std::vector<int> map_vertical_planes;
  for (int plane_nr = 0; plane_nr < map_planes.size(); ++plane_nr) {
    if (std::abs(map_planes.points[plane_nr].normal_z) > threshold_horizontal) {
      // std::cout << "horizontal map : " << plane_nr << std::endl;
      map_horizontal_planes.push_back(plane_nr);
    } else {
      map_vertical_planes.push_back(plane_nr);
    }
  }

  std::vector<int> scan_horizontal_planes;
  std::vector<int> scan_vertical_planes;
  for (int plane_nr = 0; plane_nr < scan_planes.size(); ++plane_nr) {
    if (std::abs(scan_planes.points[plane_nr].normal_z) > threshold_horizontal) {
      // std::cout << "horizontal scan : " << plane_nr << std::endl;
      scan_horizontal_planes.push_back(plane_nr);
    } else {
      scan_vertical_planes.push_back(plane_nr);
    }
  }

  // Find descriptors
  std::vector<std::vector<float>> vert_map_descriptors;
  getVerticalDescriptor(vert_map_descriptors, map_vertical_planes, parallel_threshold, map_planes);
  std::vector<std::vector<float>> horizon_map_descriptors;
  getHorizontalDescriptor(horizon_map_descriptors, map_horizontal_planes, map_planes);
  std::vector<std::vector<float>> vert_scan_descriptors;
  getVerticalDescriptor(vert_scan_descriptors, scan_vertical_planes, parallel_threshold,
                        scan_planes);
  std::vector<std::vector<float>> horizon_scan_descriptors;
  getHorizontalDescriptor(horizon_scan_descriptors, scan_horizontal_planes, scan_planes);
  std::cout << "Features calculated" << std::endl;

  // for (int map_plane_nr = 0; map_plane_nr < map_vertical_planes.size(); ++map_plane_nr) {
  //   std::cout << "map nr " << map_vertical_planes[map_plane_nr] << ": ";
  //   for (auto feature : vert_map_descriptors[map_plane_nr]) std::cout << feature << " ";
  //   std::cout << std::endl;
  // }

  // for (int map_plane_nr = 0; map_plane_nr < scan_vertical_planes.size(); ++map_plane_nr) {
  //   std::cout << "scan nr " << scan_vertical_planes[map_plane_nr] << ": ";
  //   for (auto feature : vert_scan_descriptors[map_plane_nr]) std::cout << feature << " ";
  //   std::cout << std::endl;
  // }

  // Get both scores
  std::vector<std::vector<float>> vert_assign_score_scan_to_map;
  std::vector<float> vert_assign_score_to_map;
  for (int vert_scan_plane = 0; vert_scan_plane < scan_vertical_planes.size(); ++vert_scan_plane) {
    vert_assign_score_to_map.clear();
    getDescriptorScore(vert_assign_score_to_map, vert_map_descriptors,
                       vert_scan_descriptors[vert_scan_plane]);
    vert_assign_score_scan_to_map.push_back(vert_assign_score_to_map);

    std::cout << scan_vertical_planes[vert_scan_plane] << " vertical : ";
    for (int map_plane = 0; map_plane < vert_assign_score_to_map.size(); ++map_plane)
      std::cout << map_vertical_planes[map_plane] << " " << vert_assign_score_to_map[map_plane]
                << " ";
    std::cout << std::endl;
  }

  std::vector<std::vector<float>> horiz_assign_score_scan_to_map;
  std::vector<float> horiz_assign_score_to_map;
  for (int horiz_scan_plane = 0; horiz_scan_plane < scan_horizontal_planes.size();
       ++horiz_scan_plane) {
    horiz_assign_score_to_map.clear();
    getDescriptorScore(horiz_assign_score_to_map, horizon_map_descriptors,
                       horizon_scan_descriptors[horiz_scan_plane]);
    horiz_assign_score_scan_to_map.push_back(horiz_assign_score_to_map);

    std::cout << scan_horizontal_planes[horiz_scan_plane] << " horizon : ";
    for (int map_plane = 0; map_plane < horiz_assign_score_to_map.size(); ++map_plane)
      std::cout << map_horizontal_planes[map_plane] << " " << horiz_assign_score_to_map[map_plane]
                << " ";
    std::cout << std::endl;
  }

  // Take high values as possible assignments
  float assignment_score_threshold = 0.9;
  float max_score = 0;
  std::vector<std::vector<int>> assignment_candidates(scan_planes.size(), std::vector<int>(0));
  std::vector<float> candidate_list_for_scan_plane;
  for (int vert_plane_nr = 0; vert_plane_nr < vert_assign_score_scan_to_map.size();
       ++vert_plane_nr) {
    max_score = *std::max_element(vert_assign_score_scan_to_map[vert_plane_nr].begin(),
                                  vert_assign_score_scan_to_map[vert_plane_nr].end());
    // Take map planes with enough high score as possible candidates
    candidate_list_for_scan_plane.clear();
    for (int candidate_vert_plane_map_nr = 0;
         candidate_vert_plane_map_nr < map_vertical_planes.size(); ++candidate_vert_plane_map_nr) {
      if (vert_assign_score_scan_to_map[vert_plane_nr][candidate_vert_plane_map_nr] >
          assignment_score_threshold * max_score) {
        candidate_list_for_scan_plane.push_back(map_vertical_planes[candidate_vert_plane_map_nr]);
      }
    }
    for (auto candidate : candidate_list_for_scan_plane)
      assignment_candidates[scan_vertical_planes[vert_plane_nr]].push_back(candidate);
  }

  for (int horiz_plane_nr = 0; horiz_plane_nr < horiz_assign_score_scan_to_map.size();
       ++horiz_plane_nr) {
    max_score = *std::max_element(horiz_assign_score_scan_to_map[horiz_plane_nr].begin(),
                                  horiz_assign_score_scan_to_map[horiz_plane_nr].end());
    // Take map planes with enough high score as possible candidates
    candidate_list_for_scan_plane.clear();
    for (int candidate_horiz_plane_map_nr = 0;
         candidate_horiz_plane_map_nr < map_horizontal_planes.size();
         ++candidate_horiz_plane_map_nr) {
      if (horiz_assign_score_scan_to_map[horiz_plane_nr][candidate_horiz_plane_map_nr] >
          assignment_score_threshold * max_score) {
        candidate_list_for_scan_plane.push_back(
            map_horizontal_planes[candidate_horiz_plane_map_nr]);
      }
    }
    for (auto candidate : candidate_list_for_scan_plane)
      assignment_candidates[scan_horizontal_planes[horiz_plane_nr]].push_back(candidate);
  }

  for (int i = 0; i < assignment_candidates.size(); ++i) {
    std::cout << "scan " << i << " has candidates: ";
    for (int j = 0; j < assignment_candidates[i].size(); ++j) {
      std::cout << assignment_candidates[i][j] << " ";
    }
    std::cout << std::endl;
  }

  std::vector<int> plane_assignment(scan_planes.size(), 1);
  // Assign unique assignments
  for (int scan_plane_nr = 0; scan_plane_nr < scan_planes.size(); ++scan_plane_nr) {
    if (assignment_candidates[scan_plane_nr].size() == 1)
      plane_assignment[scan_plane_nr] = assignment_candidates[scan_plane_nr][0];
  }

  // Considering geometric consistency choose best candidate for plane

  // for (int plane_nr = 0; plane_nr < scan_planes.size(); ++plane_nr) {
  //   std::cout << "Plane " << plane_nr << " from Scan matched with " << plane_assignment[plane_nr]
  //             << " from mesh" << std::endl;
  // }

  // transform_average(transformTR, plane_assignment, scan_planes, map_planes);
};

void PlaneMatchLib::getDescriptorScore(std::vector<float> &assign_score,
                                       std::vector<std::vector<float>> map_descriptors,
                                       std::vector<float> scan_descriptor) {
  float deviation_offset = 1;
  // Get error of all assingments
  assign_score.clear();
  std::vector<float> feature_errors;
  std::vector<float> min_current_error;
  float current_score;
  for (int map_plane_nr = 0; map_plane_nr < map_descriptors.size(); ++map_plane_nr) {
    min_current_error.clear();
    for (auto scan_feature : scan_descriptor) {
      feature_errors.clear();
      for (auto map_feature : map_descriptors[map_plane_nr]) {
        feature_errors.push_back(std::abs(scan_feature - map_feature));
      }
      min_current_error.push_back(*std::min_element(feature_errors.begin(), feature_errors.end()));
    }
    current_score = 0;
    for (auto feature_error : min_current_error) {
      current_score = current_score + std::max(-1.0, (deviation_offset - pow(feature_error, 2)));
    }
    assign_score.push_back(current_score);
  }
}

void PlaneMatchLib::getVerticalDescriptor(std::vector<std::vector<float>> &rel_plane_descriptor,
                                          std::vector<int> consider_planes,
                                          float parallel_threshold,
                                          const pcl::PointCloud<pcl::PointNormal> planes) {
  Eigen::Vector2f point_plane;
  Eigen::Vector2f point_plane_dir;
  Eigen::Vector2f point_pair_plane;
  Eigen::Vector2f point_pair_plane_dir;
  Eigen::Hyperplane<float, 2> line_plane;
  Eigen::Hyperplane<float, 2> line_pair_plane;
  std::vector<float> current_descriptor;
  float current_feature;

  for (auto plane : consider_planes) {
    // Needs to be changed for rotation in pitch or roll (mult with cos)
    point_plane(0) = planes.points[plane].x;
    point_plane(1) = planes.points[plane].y;
    point_plane_dir(0) = planes.points[plane].normal_x;
    point_plane_dir(1) = planes.points[plane].normal_y;
    line_plane = Eigen::Hyperplane<float, 2>::Through(point_plane, point_plane + point_plane_dir);
    current_descriptor.clear();
    for (auto pair_plane : consider_planes) {
      point_pair_plane(0) = planes.points[pair_plane].x;
      point_pair_plane(1) = planes.points[pair_plane].y;
      point_pair_plane_dir(0) = planes.points[pair_plane].normal_y;
      point_pair_plane_dir(1) = -planes.points[pair_plane].normal_x;
      if (std::abs(point_plane_dir.transpose() * point_pair_plane_dir) < parallel_threshold) {
        line_pair_plane = Eigen::Hyperplane<float, 2>::Through(
            point_pair_plane, point_pair_plane + point_pair_plane_dir);
        current_feature =
            (point_plane - line_plane.intersection(line_pair_plane)).transpose() * point_plane_dir;
        current_descriptor.push_back(current_feature);
      } else {
        continue;
      }
    }
    sort(current_descriptor.begin(), current_descriptor.end());
    rel_plane_descriptor.push_back(current_descriptor);
  }
}

void PlaneMatchLib::getHorizontalDescriptor(std::vector<std::vector<float>> &rel_plane_descriptor,
                                            std::vector<int> consider_planes,
                                            const pcl::PointCloud<pcl::PointNormal> planes) {
  std::vector<float> current_descriptor;
  for (auto plane : consider_planes) {
    current_descriptor.clear();
    for (auto pair_plane : consider_planes) {
      current_descriptor.push_back(planes.points[pair_plane].z - planes.points[plane].z);
    }
    sort(current_descriptor.begin(), current_descriptor.end());
    rel_plane_descriptor.push_back(current_descriptor);
  }
}

void PlaneMatchLib::load_example_sol(float (&transformTR)[7],
                                     const pcl::PointCloud<pcl::PointNormal> scan_planes,
                                     const pcl::PointCloud<pcl::PointNormal> map_planes,
                                     ros::NodeHandle &nh_private) {
  std::vector<int> plane_assignment(scan_planes.size(), 0);
  // // Real data
  // // Solution to example
  // plane_assignment[0] = 14;
  // plane_assignment[1] = 21;
  // plane_assignment[2] = 1;
  // plane_assignment[3] = 0;
  // plane_assignment[4] = 2;
  // plane_assignment[5] = 0;
  // plane_assignment[6] = 14;
  // plane_assignment[7] = 4;

  // Simulated data
  // Simulated data
  // Solution to example with rot 0,0,0,1
  // plane_assignment[0] = 14;
  // plane_assignment[1] = 2;
  // plane_assignment[2] = 0;
  // plane_assignment[3] = 2;
  // plane_assignment[4] = 1;
  // plane_assignment[5] = 0;
  // plane_assignment[6] = 2;
  // plane_assignment[7] = 0;

  // Solution to example with rot 0 0 0.3428978 0.9393727
  plane_assignment[0] = 14;
  plane_assignment[1] = 2;
  plane_assignment[2] = 0;
  plane_assignment[3] = 2;
  plane_assignment[4] = 0;
  plane_assignment[5] = 1;
  plane_assignment[6] = 0;
  plane_assignment[7] = 2;

  transform_average(transformTR, plane_assignment, scan_planes, map_planes);
}

void PlaneMatchLib::transform_average(float (&transformTR)[7], std::vector<int> plane_assignment,
                                      const pcl::PointCloud<pcl::PointNormal> scan_planes,
                                      const pcl::PointCloud<pcl::PointNormal> map_planes) {
  std::cout << "Calculate Transformation via plane correspondences" << std::endl;
  // Calculate average quaternion (Map to Lidar), R_Map_to_Lidar
  Eigen::MatrixXd all_quat(4, scan_planes.size());
  Eigen::Quaterniond current_quat;
  Eigen::Vector3d map_normal;
  Eigen::Vector3d scan_normal;
  for (int plane_nr; plane_nr < scan_planes.size(); ++plane_nr) {
    scan_normal(0) = scan_planes.points[plane_nr].normal_x;
    scan_normal(1) = scan_planes.points[plane_nr].normal_y;
    scan_normal(2) = scan_planes.points[plane_nr].normal_z;
    map_normal(0) = map_planes.points[plane_assignment[plane_nr]].normal_x;
    map_normal(1) = map_planes.points[plane_assignment[plane_nr]].normal_y;
    map_normal(2) = map_planes.points[plane_assignment[plane_nr]].normal_z;
    current_quat = Eigen::Quaterniond::FromTwoVectors(scan_normal, map_normal);
    all_quat(0, plane_nr) = current_quat.w();
    all_quat(1, plane_nr) = current_quat.x();
    all_quat(2, plane_nr) = current_quat.y();
    all_quat(3, plane_nr) = current_quat.z();
  }
  // M with weight 1
  Eigen::Matrix4d quat_quat = all_quat * all_quat.transpose();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigensolver(quat_quat);
  Eigen::Vector4d eigenvalues = eigensolver.eigenvalues().col(0);

  int max_ev_index = 0;
  for (int i = 0; i < all_quat.rows(); ++i) {
    if (eigenvalues[i] > eigenvalues[max_ev_index]) {
      max_ev_index = i;
    }
  }
  Eigen::Vector4d res_quat = eigensolver.eigenvectors().col(max_ev_index);

  // res_quat[0] = 0.939;
  // res_quat[1] = 0;
  // res_quat[2] = 0;
  // res_quat[3] = 0.34;

  // Assign to solution
  transformTR[3] = res_quat[0];
  transformTR[4] = res_quat[1];
  transformTR[5] = res_quat[2];
  transformTR[6] = res_quat[3];

  // Find translation
  pcl::PointCloud<pcl::PointNormal> copy_scan_planes;
  pcl::copyPointCloud(scan_planes, copy_scan_planes);
  Eigen::Matrix4f res_rotation = Eigen::Matrix4f::Identity();
  Eigen::Quaternionf q(transformTR[3], transformTR[4], transformTR[5], transformTR[6]);
  res_rotation.block(0, 0, 3, 3) = q.matrix();
  pcl::transformPointCloud(copy_scan_planes, copy_scan_planes, res_rotation);

  double mean_translation_x = 0;
  double mean_translation_y = 0;
  double mean_translation_z = 0;
  int nr_planes_x = 0;
  int nr_planes_y = 0;
  int nr_planes_z = 0;
  int map_index;
  for (int plane_nr = 0; plane_nr < scan_planes.size(); ++plane_nr) {
    map_index = plane_assignment[plane_nr];
    if (std::abs(map_planes.points[map_index].normal_x) > 0.5) {
      mean_translation_x =
          map_planes.points[map_index].x - copy_scan_planes.points[plane_nr].x + mean_translation_x;
      ++nr_planes_x;
    }
    if (std::abs(map_planes.points[map_index].normal_y) > 0.5) {
      mean_translation_y =
          map_planes.points[map_index].y - copy_scan_planes.points[plane_nr].y + mean_translation_y;
      ++nr_planes_y;
    }
    if (std::abs(map_planes.points[plane_nr].normal_z) > 0.5) {
      mean_translation_z =
          map_planes.points[map_index].z - copy_scan_planes.points[plane_nr].z + mean_translation_z;
      ++nr_planes_z;
    }
  }
  if (nr_planes_x != 0) {
    transformTR[0] = mean_translation_x / nr_planes_x;
  } else {
    std::cout << "Couldn't find translation x as plane with normal x is missing" << std::endl;
    transformTR[0] = 0;
  }
  if (nr_planes_y != 0) {
    transformTR[1] = mean_translation_y / nr_planes_y;
  } else {
    std::cout << "Couldn't find translation y as plane with normal y is missing" << std::endl;
    transformTR[1] = 0;
  }
  if (nr_planes_z != 0) {
    transformTR[2] = mean_translation_z / nr_planes_z;
  } else {
    std::cout << "Couldn't find translation z as plane with normal z is missing" << std::endl;
    transformTR[2] = 0;
  }
};
