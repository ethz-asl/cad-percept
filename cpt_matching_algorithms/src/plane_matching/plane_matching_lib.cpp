#include "plane_matching/plane_matching_lib.h"

using namespace cad_percept::cgal;

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
                                    const pcl::PointCloud<pcl::PointNormal> map_planes) {
  float threshold_horizontal = 0.8;
  float parallel_threshold = 0.5;
  std::cout << "////  Plane Descriptor Started  ////" << std::endl;

  std::vector<std::vector<Eigen::Vector3d>> scan_intersection_points;
  std::vector<std::vector<std::array<int, 3>>> used_scan_planes;
  std::vector<std::vector<Eigen::Vector3d>> map_intersection_points;
  std::vector<std::vector<std::array<int, 3>>> used_map_planes;

  // Find intersection points of intersection line of two planes with one other plane
  //  getPlaneIntersectionPoints(scan_intersection_points, used_scan_planes, parallel_threshold,
  //                             scan_planes);
  getPlaneIntersectionPoints(map_intersection_points, used_map_planes, parallel_threshold,
                             map_planes);

  // getCornerDescriptors();
  // getCornerDescriptors();

  // getCornerMatchScore();
  // getCornerMatchScore();
};

void PlaneMatchLib::getPlaneIntersectionPoints(
    std::vector<std::vector<Eigen::Vector3d>> &tot_plane_intersections,
    std::vector<std::vector<std::array<int, 3>>> &used_planes, float parallel_threshold,
    const pcl::PointCloud<pcl::PointNormal> planes) {
  Plane cgal_plane;
  Plane cgal_pair_plane;
  Line intersection_line;

  Plane cgal_candidate_plane;
  Point intersection_point;
  std::array<int, 3> used_plane;

  tot_plane_intersections.clear();
  tot_plane_intersections =
      std::vector<std::vector<Eigen::Vector3d>>(planes.size(), std::vector<Eigen::Vector3d>(0));
  used_planes.clear();
  used_planes = std::vector<std::vector<std::array<int, 3>>>(planes.size(),
                                                             std::vector<std::array<int, 3>>(0));

  // Get point on intersection plane
  int plane_nr = 0;
  int pair_plane_nr = 0;
  int candidate_nr = 0;
  CGAL::Object result_intersection_line;
  CGAL::Object result_intersection_point;

  for (auto plane : planes) {
    cgal_plane = Plane(Point(plane.x, plane.y, plane.z),
                       Vector(plane.normal_x, plane.normal_y, plane.normal_z));
    pair_plane_nr = 0;
    for (auto pair_plane : planes) {
      cgal_pair_plane =
          Plane(Point(pair_plane.x, pair_plane.y, pair_plane.z),
                Vector(pair_plane.normal_x, pair_plane.normal_y, pair_plane.normal_z));
      // Make sure planes are not too parallel
      if (cgal_plane.orthogonal_vector() * cgal_pair_plane.orthogonal_vector() > parallel_threshold)
        continue;
      result_intersection_line = CGAL::intersection(cgal_plane, cgal_pair_plane);
      // Skip plane pair if there is no intersection_line (should not happen)
      if (!CGAL::assign(intersection_line, result_intersection_line)) continue;
      candidate_nr = 0;
      for (auto candidate_plane : planes) {
        cgal_candidate_plane = Plane(
            Point(candidate_plane.x, candidate_plane.y, candidate_plane.z),
            Vector(candidate_plane.normal_x, candidate_plane.normal_y, candidate_plane.normal_z));
        result_intersection_point = CGAL::intersection(cgal_candidate_plane, intersection_line);
        if (!CGAL::assign(intersection_point, result_intersection_point)) continue;

        tot_plane_intersections[candidate_nr].push_back(
            Eigen::Vector3d((double)(intersection_point.x()), (double)(intersection_point.y()),
                            (double)(intersection_point.z())));
        used_plane = {candidate_nr, pair_plane_nr, plane_nr};
        used_planes[candidate_nr].push_back(used_plane);

        std::cout << "map: " << (double)(intersection_point.x()) << " "
                  << (double)(intersection_point.y()) << " " << (double)(intersection_point.z())
                  << std::endl;
        std::cout << " " << used_plane[0] << " " << used_plane[1] << " " << used_plane[2]
                  << std::endl;

        candidate_nr++;
      }
      pair_plane_nr++;
    }
    plane_nr++;
  }
}

//   for (auto plane : planes) {
//     plane_normal = Eigen::Vector3d(plane.normal_x, plane.normal_y, plane.normal_z);
//     pair_plane_nr = 0;
//     for (auto pair_plane : planes) {
//       pair_plane_normal =
//           Eigen::Vector3d(pair_plane.normal_x, pair_plane.normal_y, pair_plane.normal_z);
//       if (plane_normal.dot(pair_plane_normal) > parallel_threshold) {
//         continue;
//       }
//       normal_of_common_plane = plane_normal.cross(pair_plane_normal);
//       intersect_plane = normal_of_common_plane.cross(plane_normal);
//       intersect_pair_plane = normal_of_common_plane.cross(pair_plane_normal);

//       // Cite fromula
//       // Get point on intersection lines
//       float scale_vector_one = intersect_pair_plane.cross(
//           Eigen::Vector3d((double)(plane.x - pair_plane.x), (double)(plane.y - pair_plane.y),
//                           (double)(plane.z - pair_plane.z)));
//       float scale_vector_two = intersect_pair_plane.cross(intersect_plane);

//       distance_to_edge = scale_vector_one.norm() / scale_vector_two.norm();
//       if (scale_vector_one.dot(scale_vector_two) < 0) {
//         distance_to_edge = -distance_to_edge;
//       }
//       edge_point = Eigen::Vector3d(plane.x, plane.y, plane.z + distance_to_edge *
//       intersect_plane);

//       intersection_line = Line(Point(edge_point(0), edge_point(1), edge_point(2)),
//                                eigenVectorToCgalPoint(normal_of_common_plane));
//       candidate_nr = 0;
//       for (auto intersection_plane : planes) {
//         // if the planes contribute to the line, skip
//         if (candidate_nr == plane_nr || candidate_nr == pair_plane_nr) continue;
//         candidate_plane =
//             Plane(Point(intersection_plane.x, intersection_plane.y, intersection_plane.z,
//                         Vector(intersection_plane.normal_x, intersection_plane.normal_y,
//                                intersection_plane.normal_z)));
//         result = CGAL::intersection(candidate_plane, intersection_line);
//         if (CGAL::assign(intersection_point, result)) {
//           used_plane = {candidate_nr, pair_plane_nr, plane_nr};
//           used_planes[candidate_nr].push_back(used_plane);
//           tot_plane_descriptor[candidate_nr].push_back(cgalVectorToEigenVector(intersection_point));
//         } else {
//           // No unique intersection, skip
//           continue;
//         }
//         candidate_nr++;
//       }
//       pair_plane_nr++;
//     }
//     plane_nr++;
//   }
// }

// void PlaneMatchLib::getDescriptorScore(std::vector<float> &assign_score,
//                                        std::vector<std::vector<float>> map_descriptors,
//                                        std::vector<float> scan_descriptor) {
//   float deviation_offset = 1;
// }

// void PlaneMatchLib::getPlaneDescriptor(std::vector<std::vector<float>> descriptor_list,
//                                        std::vector<std::vector<Eigen::Vector3d>> corner_points)
//                                        {}

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
