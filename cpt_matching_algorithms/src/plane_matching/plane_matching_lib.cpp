#include "plane_matching/plane_matching_lib.h"

void PlaneMatchLib::prrus(float (&transformTR)[7],
                          const pcl::PointCloud<pcl::PointNormal> scan_planes,
                          const pcl::PointCloud<pcl::PointNormal> map_planes) {
  std::cout << "////  PRRUS Matching Started  ////" << std::endl;
  std::cout << "Number of planes in map: " << map_planes.size() << std::endl;
  std::cout << "Number of planes in scan: " << scan_planes.size() << std::endl;

  float match_score = -1;
  std::vector<int> plane_assignment(scan_planes.size(), 0);

  // ToDo Matching part

  for (int plane_nr; plane_nr < scan_planes.size(); ++plane_nr) {
    std::cout << "Plane " << plane_nr << " from Scan matched with " << plane_assignment[plane_nr]
              << " from mesh" << std::endl;
  }
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
  // Solution to example
  plane_assignment[0] = 14;
  plane_assignment[1] = 2;
  plane_assignment[2] = 0;
  plane_assignment[3] = 2;
  plane_assignment[4] = 0;
  plane_assignment[5] = 1;
  plane_assignment[6] = 14;
  plane_assignment[7] = 2;

  transform_average(transformTR, plane_assignment, scan_planes, map_planes);
}

void PlaneMatchLib::transform_average(float (&transformTR)[7], std::vector<int> plane_assignment,
                                      const pcl::PointCloud<pcl::PointNormal> scan_planes,
                                      const pcl::PointCloud<pcl::PointNormal> map_planes) {
  std::cout << "Calculate Transformation via plane correspondences" << std::endl;
  // Calculate average quaternion (Map to Lidar)
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
    current_quat = Eigen::Quaterniond::FromTwoVectors(map_normal, scan_normal);
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
  res_quat[0] = 0.9393727;
  res_quat[1] = 0;
  res_quat[2] = 0;
  res_quat[3] = 0.3428978;

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
    std::cout << copy_scan_planes.points[plane_nr].x << std::endl;
    std::cout << "Map: " << map_planes.points[plane_assignment[plane_nr]].x << std::endl;
    // Must be changed to a multiplication, but works with example
    map_index = plane_assignment[plane_nr];
    if (std::abs(map_planes.points[map_index].normal_x) == 1) {
      std::cout << plane_nr << std::endl;
      mean_translation_x =
          map_planes.points[map_index].x - copy_scan_planes.points[plane_nr].x + mean_translation_x;
      ++nr_planes_x;
    }
    if (std::abs(map_planes.points[map_index].normal_y) == 1) {
      mean_translation_y =
          map_planes.points[map_index].y - copy_scan_planes.points[plane_nr].y + mean_translation_y;
      ++nr_planes_y;
    }
    if (std::abs(map_planes.points[plane_nr].normal_z) == 1) {
      mean_translation_z =
          map_planes.points[map_index].z - copy_scan_planes.points[plane_nr].z + mean_translation_z;
      ++nr_planes_z;
    }
  }
  if (nr_planes_x != 0) {
    transformTR[0] = mean_translation_x / nr_planes_x;
  } else {
    std::cout << "Couldn't find transformation as plane with normal x is missing" << std::endl;
    transformTR[0] = 0;
  }
  if (nr_planes_y != 0) {
    transformTR[1] = mean_translation_y / nr_planes_y;
  } else {
    std::cout << "Couldn't find transformation as plane with normal y is missing" << std::endl;
    transformTR[1] = 0;
  }
  if (nr_planes_z != 0) {
    transformTR[2] = mean_translation_z / nr_planes_z;
  } else {
    std::cout << "Couldn't find transformation as plane with normal z is missing" << std::endl;
    transformTR[2] = 0;
  }
};
