#include "plane_matching/plane_matching.h"

using namespace cad_percept::cgal;

namespace cad_percept {
namespace matching_algorithms {

PlaneMatch::prrusConfig PlaneMatch::loadPrrusConfigFromServer() {
  ros::NodeHandle nh_private("~");

  prrusConfig config;
  config.translation_penalty = nh_private.param<float>("PRRUSTranErrorPenalty", 20);
  config.tol = nh_private.param<float>("PRRUSTolToPlane", 0.05);

  return config;
}

float PlaneMatch::prrus(Transformation &transform,
                        const pcl::PointCloud<pcl::PointNormal> &scan_planes,
                        BoundedPlanes map_planes, const prrusConfig &config) {
  std::cout << "////////////////////////////////////////////////" << std::endl;
  std::cout << "////              PRRUS Started             ////" << std::endl;
  std::cout << "////////////////////////////////////////////////" << std::endl;

  pcl::PointCloud<pcl::PointNormal> map_planes_pc = map_planes.getPlaneCentroidsAndNormals();

  // Find groups of orthogonal planes in map
  std::vector<Eigen::Vector3i> orthogonal_groups_in_map;
  createOrthogonalSets(orthogonal_groups_in_map, map_planes_pc, false);
  if (orthogonal_groups_in_map.size() == 0) {
    std::cout
        << "Can not find transformation, as no group of orthogonal planes in map could be found"
        << std::endl;
    return -1;
  } else {
    std::cout << "Found " << orthogonal_groups_in_map.size()
              << " groups of orthogonal planes in map" << std::endl;
  }

  // Find groups of orthogonal planes in scan
  std::vector<Eigen::Vector3i> orthogonal_groups_in_scan;
  createOrthogonalSets(orthogonal_groups_in_scan, scan_planes, true);
  if (orthogonal_groups_in_scan.size() == 0) {
    std::cout
        << "Can not find transformation, as no group of orthogonal planes in scan could be found"
        << std::endl;
    return -1;
  } else {
    std::cout << "Found " << orthogonal_groups_in_scan.size()
              << " groups of orthogonal planes in scan" << std::endl;
  }

  // Create assignments which are consistent considering rotation
  std::vector<std::vector<std::pair<size_t, size_t>>> assignments;
  std::vector<Eigen::Quaterniond> rotations_of_assignments;
  checkRotationConsistency(assignments, rotations_of_assignments, orthogonal_groups_in_scan,
                           orthogonal_groups_in_map, scan_planes, map_planes_pc);
  if (assignments.size() == 0) {
    std::cout << "Can not find transformation, as no consistent rotation could be found"
              << std::endl;
    return -1;
  } else {
    std::cout << "Found " << assignments.size() << " possible assignments" << std::endl;
  }

  // Find translation and apply cost function on left free assignments
  float min_error = FLT_MAX;
  float actual_error = 0;
  Transformation actual_transformation;
  int assignment_nr = 0;
  std::vector<float> actual_result;
  for (auto candidate_assignment : assignments) {
    getTranslationError(candidate_assignment, actual_transformation, actual_error,
                        rotations_of_assignments[assignment_nr], scan_planes, map_planes,
                        config.translation_penalty, config.tol);
    if (actual_error < min_error) {
      transform = actual_transformation;
      min_error = actual_error;
      std::cout << "error: " << min_error << std::endl;
    }
    ++assignment_nr;
  }
  std::cout << "final error: " << min_error << std::endl;
  return min_error;
};

void PlaneMatch::createOrthogonalSets(std::vector<Eigen::Vector3i> &orthogonal_groups,
                                      const pcl::PointCloud<pcl::PointNormal> &planes,
                                      bool add_permutation) {
  Eigen::Vector3d plane_normal[3];
  Eigen::Vector3d cross_product;
  orthogonal_groups.clear();

  for (int i = 0; i < ((int)planes.size() - 2); ++i) {
    plane_normal[0] = NormalToEigenVector(planes.points[i]);
    for (int j = i + 1; j < ((int)planes.size() - 1); ++j) {
      plane_normal[1] = NormalToEigenVector(planes.points[j]);
      // Continue if normals are collinear
      cross_product = plane_normal[0].cross(plane_normal[1]);
      if (cross_product.norm() < 0.8) continue;
      for (int k = j + 1; k < planes.size(); ++k) {
        plane_normal[2] = NormalToEigenVector(planes.points[k]);
        if (std::abs(cross_product.dot(plane_normal[2])) > 0.8) {
          if (add_permutation) {
            // If orthogonal set, save combination (all permutatios)
            orthogonal_groups.push_back(Eigen::Vector3i(i, j, k));
            orthogonal_groups.push_back(Eigen::Vector3i(i, k, j));
            orthogonal_groups.push_back(Eigen::Vector3i(j, i, k));
            orthogonal_groups.push_back(Eigen::Vector3i(j, k, i));
            orthogonal_groups.push_back(Eigen::Vector3i(k, i, j));
            orthogonal_groups.push_back(Eigen::Vector3i(k, j, i));
          } else {
            // If orthogonal set, save combination
            orthogonal_groups.push_back(Eigen::Vector3i(i, j, k));
          }
        }
      }
    }
  }
};

void PlaneMatch::checkRotationConsistency(
    std::vector<std::vector<std::pair<size_t, size_t>>> &assignments,
    std::vector<Eigen::Quaterniond> &rotations_of_assignments,
    const std::vector<Eigen::Vector3i> &orthogonal_groups_in_scan,
    const std::vector<Eigen::Vector3i> &orthogonal_groups_in_map,
    const pcl::PointCloud<pcl::PointNormal> &scan_planes,
    const pcl::PointCloud<pcl::PointNormal> &map_planes) {
  // Create assignments which are consistent considering rotation
  Eigen::Quaterniond current_quat;
  Eigen::Hyperplane<double, 3> second_rotation_plane;
  Eigen::Vector3d scan_normal[3];
  Eigen::Vector3d map_normal[3];
  Eigen::Vector3d reprojected_map_normal;
  bool is_valid_assignment;
  std::vector<std::pair<size_t, size_t>> current_assignment;

  for (auto scan_orth_group : orthogonal_groups_in_scan) {
    for (auto map_orth_group : orthogonal_groups_in_map) {
      scan_normal[0] = NormalToEigenVector(scan_planes.points[scan_orth_group[0]]);
      scan_normal[1] = NormalToEigenVector(scan_planes.points[scan_orth_group[1]]);
      scan_normal[2] = NormalToEigenVector(scan_planes.points[scan_orth_group[2]]);
      map_normal[0] = NormalToEigenVector(map_planes.points[map_orth_group[0]]);
      map_normal[1] = NormalToEigenVector(map_planes.points[map_orth_group[1]]);
      map_normal[2] = NormalToEigenVector(map_planes.points[map_orth_group[2]]);

      // Calculate corresponding quaternion (algorithm from
      // https://stackoverflow.com/questions/19445934/quaternion-from-two-vector-pairs)
      current_quat = Eigen::Quaterniond::FromTwoVectors(scan_normal[1], map_normal[1]);
      reprojected_map_normal = current_quat.inverse() * map_normal[0];
      second_rotation_plane =
          Eigen::Hyperplane<double, 3>(scan_normal[1], Eigen::Vector3d(0, 0, 0));
      current_quat = (current_quat * Eigen::Quaterniond::FromTwoVectors(
                                         second_rotation_plane.projection(scan_normal[0]),
                                         second_rotation_plane.projection(reprojected_map_normal)))
                         .normalized();
      // Rotate scan_normals
      is_valid_assignment = true;
      current_assignment.clear();
      for (int i = 0; i < 3 && is_valid_assignment; ++i) {
        scan_normal[i] = current_quat * scan_normal[i];
        is_valid_assignment = is_valid_assignment && (scan_normal[i].dot(map_normal[i]) > 0.8);
        current_assignment.push_back(std::make_pair(scan_orth_group[i], map_orth_group[i]));
      }
      if (is_valid_assignment) {
        // Add valid assignment
        std::sort(current_assignment.begin(), current_assignment.end(),
                  [](const std::pair<size_t, size_t> &l, const std::pair<size_t, size_t> &r) {
                    return l.first > r.first;
                  });
        assignments.push_back(current_assignment);
        rotations_of_assignments.push_back(current_quat);
      }
    }
  }
};

void PlaneMatch::getTranslationError(std::vector<std::pair<size_t, size_t>> plane_assignment,
                                     Transformation &transform, float &transl_error,
                                     Eigen::Quaterniond rotation,
                                     const pcl::PointCloud<pcl::PointNormal> &scan_planes,
                                     BoundedPlanes map_planes, float translation_penalty,
                                     float tol) {
  transl_error = 0;

  // Get all map planes
  pcl::PointCloud<pcl::PointNormal> map_planes_pc = map_planes.getPlaneCentroidsAndNormals();
  // Save the three reference planes in seperated pc for better handling
  pcl::PointCloud<pcl::PointNormal> reference_map_planes;
  pcl::PointCloud<pcl::PointNormal> reference_scan_planes;

  // Find translation (only with assignment)
  for (int i = 0; i < plane_assignment.size(); ++i) {
    reference_scan_planes.push_back(scan_planes.points[plane_assignment[i].first]);
    reference_map_planes.push_back(map_planes_pc.points[plane_assignment[i].second]);
  }

  Eigen::Matrix4d eigen_transf = Eigen::Matrix4d::Identity();
  eigen_transf.block(0, 0, 3, 3) = rotation.matrix();
  pcl::transformPointCloud(reference_scan_planes, reference_scan_planes, eigen_transf);

  // Calculate translation
  Eigen::Vector3d translation(0, 0, 0);
  for (int plane_nr = 0; plane_nr < reference_scan_planes.size(); ++plane_nr) {
    translation += (PointToEigenVector(reference_map_planes.points[plane_nr]) -
                    PointToEigenVector(reference_scan_planes.points[plane_nr]))
                       .dot(NormalToEigenVector(reference_map_planes.points[plane_nr])) *
                   NormalToEigenVector(reference_map_planes.points[plane_nr]);
  }

  // Get transformation matrix
  eigen_transf.block(0, 3, 3, 1) = translation;

  pcl::PointCloud<pcl::PointNormal> transformed_scan_planes;
  pcl::transformPointCloudWithNormals(scan_planes, transformed_scan_planes, eigen_transf);

  // Get assignment error of found transformation
  std::vector<float> actual_translation_error;
  int map_plane_nr = 0;
  int scan_plane_nr = 0;
  for (auto scan_plane : transformed_scan_planes) {
    map_plane_nr = 0;
    actual_translation_error.clear();
    if (scan_plane_nr == plane_assignment.back().first) {
      // Take reference if plane is in matching
      // Only accept point as possible match if projection lies on this plane
      if (map_planes.isProjectionOfPointOnPlane(PointToEigenVector(scan_plane),
                                                plane_assignment.back().second, tol)) {
        // Add plane as (single) candidate to error calculation
        actual_translation_error.push_back(std::abs(
            (PointToEigenVector(scan_plane) -
             PointToEigenVector(map_planes_pc.points[plane_assignment.back().second]))
                .dot(NormalToEigenVector(map_planes_pc.points[plane_assignment.back().second]))));
      }
      plane_assignment.pop_back();
    } else {  // search closest plane
      // Only accept plane if it has the same normal
      for (auto map_plane : map_planes_pc) {
        if (NormalToEigenVector(map_plane).dot(NormalToEigenVector(scan_plane)) < 0.8) {
          map_plane_nr++;
          continue;
        }
        // Only accept point as possible match if projection lies on this plane
        if (!map_planes.isProjectionOfPointOnPlane(PointToEigenVector(scan_plane), map_plane_nr,
                                                   tol)) {
          map_plane_nr++;
          continue;
        }
        // Push back possible smallest distance to one of the map planes
        actual_translation_error.push_back(
            std::abs((PointToEigenVector(scan_plane) - PointToEigenVector(map_plane))
                         .dot(NormalToEigenVector(map_plane))));
        map_plane_nr++;
      }
    }
    if (actual_translation_error.size() != 0) {
      transl_error +=
          *std::min_element(actual_translation_error.begin(), actual_translation_error.end());
    } else {
      transl_error += translation_penalty;
    }
    ++scan_plane_nr;
  }
  // Save Eigen Transformation to CGAL Transformation
  eigenTransformationToCgalTransformation(eigen_transf, &transform);
};

Eigen::Vector3d PlaneMatch::PointToEigenVector(pcl::PointNormal point) {
  return Eigen::Vector3d(point.x, point.y, point.z);
}
Eigen::Vector3d PlaneMatch::NormalToEigenVector(pcl::PointNormal point) {
  return Eigen::Vector3d(point.normal_x, point.normal_y, point.normal_z);
}

}  // namespace matching_algorithms
}  // namespace cad_percept