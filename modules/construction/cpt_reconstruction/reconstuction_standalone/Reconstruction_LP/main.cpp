#include <pcl/ModelCoefficients.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/covariance_sampling.h>
#include <pcl/filters/normal_space.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/octree/octree_search.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <Eigen/Core>
#include <cmath>
#include <exception>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <utility>

//#include <pcl/common/impl/intersections.hpp>
#include <pcl/PCLHeader.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PolygonMesh.h>
#include <pcl/Vertices.h>
#include <pcl/common/distances.h>
#include <pcl/common/intersections.h>
#include <pcl/common/io.h>
#include <pcl/common/pca.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/geometry/quad_mesh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/simplification_remove_unused_vertices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#define CGAL_USE_SCIP
#include <CGAL/SCIP_mixed_integer_program_traits.h>
typedef CGAL::SCIP_mixed_integer_program_traits<double> MIP_Solver;
typedef typename MIP_Solver::Variable Variable;
typedef typename MIP_Solver::Linear_objective Linear_objective;
typedef typename MIP_Solver::Linear_constraint Linear_constraint;

// Source:https://github.com/tttamaki/ICP-test/blob/master/src/icp3_with_normal_iterative_view.cpp
void addNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                pcl::PointCloud<pcl::Normal>::Ptr normals, int k) {
  pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  searchTree->setInputCloud(cloud);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
  normalEstimator.setInputCloud(cloud);
  normalEstimator.setSearchMethod(searchTree);
  normalEstimator.setKSearch(k);
  normalEstimator.compute(*normals);
}

void combineMeshes(const pcl::PolygonMesh &mesh, pcl::PolygonMesh &mesh_all) {
  // pcl::PolygonMesh::concatenate(mesh_all, mesh); ???
  // Source:
  // https://github.com/PointCloudLibrary/pcl/blob/master/common/include/pcl/PolygonMesh.h
  mesh_all.header.stamp = std::max(mesh_all.header.stamp, mesh.header.stamp);

  const auto point_offset = mesh_all.cloud.width * mesh_all.cloud.height;

  // Transform them to PointXYZ and back to PCLPointCloud (to remove redundant
  // fields)
  pcl::PointCloud<pcl::PointXYZ> mesh_all_cloud;
  pcl::PointCloud<pcl::PointXYZ> mesh_cloud;
  pcl::fromPCLPointCloud2(mesh_all.cloud, mesh_all_cloud);
  pcl::fromPCLPointCloud2(mesh.cloud, mesh_cloud);
  pcl::PCLPointCloud2 mesh_all_cloud2;
  pcl::PCLPointCloud2 mesh_cloud2;
  pcl::toPCLPointCloud2(mesh_cloud, mesh_cloud2);
  pcl::toPCLPointCloud2(mesh_all_cloud, mesh_all_cloud2);

  // Conatenate point clouds
  pcl::PCLPointCloud2 new_cloud;
  pcl::concatenatePointCloud(mesh_all_cloud2, mesh_cloud2, new_cloud);
  mesh_all.cloud = new_cloud;

  // Concatenate polygons
  std::transform(
      mesh.polygons.begin(), mesh.polygons.end(),
      std::back_inserter(mesh_all.polygons), [point_offset](auto polygon) {
        std::transform(polygon.vertices.begin(), polygon.vertices.end(),
                       polygon.vertices.begin(),
                       [point_offset](auto &point_idx) {
                         return point_idx + point_offset;
                       });
        return polygon;
      });
}

void planeFromIdx(Eigen::Vector4f &result, int idx,
                  std::vector<Eigen::Vector3d> &plane_normals,
                  std::vector<double> &plane_d) {
  Eigen::Vector3d normals = plane_normals.at(idx);
  double d = plane_d.at(idx);
  result(0) = (float)normals.x();
  result(1) = (float)normals.y();
  result(2) = (float)normals.z();
  result(3) = (float)d;
}

void computeAllPlanes(std::vector<::pcl::Vertices> &faces_model,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr vertices_model,
                      std::vector<Eigen::Vector3d> &plane_normals,
                      std::vector<double> &plane_d, std::vector<double> &area,
                      bool do_normal_correction = true, double eps = 0.1) {
  for (int i = 0; i < faces_model.size(); i++) {
    std::vector<uint32_t> vertices = faces_model.at(i).vertices;
    if (vertices.size() != 3) {
      std::cout << "Warning: Non tria element" << std::endl;
    }
    pcl::PointXYZ p1 = (*vertices_model)[vertices.at(0)];
    pcl::PointXYZ p2 = (*vertices_model)[vertices.at(1)];
    pcl::PointXYZ p3 = (*vertices_model)[vertices.at(2)];
    Eigen::Vector3d v1(p1.x, p1.y, p1.z);
    Eigen::Vector3d v2(p2.x, p2.y, p2.z);
    Eigen::Vector3d v3(p3.x, p3.y, p3.z);
    Eigen::Vector3d n = (v2 - v1).cross(v3 - v1);
    area.push_back(n.norm());

    // Normal correction up to 5 degree
    n.normalize();
    if (do_normal_correction) {
      for (int i = 0; i < 3; i++) {
        if (n(i) < -(1. - eps)) {
          n(i) = -1.;
        } else if (n(i) > -eps && n(i) < eps) {
          n(i) = 0.;
        } else if (n(i) > (1. - eps)) {
          n(i) = 1.;
        }
        n.normalize();
      }
    }

    plane_normals.push_back(n);
    double d = -(n.x() * p1.x + n.y() * p1.y + n.z() * p1.z);
    plane_d.push_back(d);
  }
}

void flagDuplicatedPlanes(std::vector<::pcl::Vertices> &faces_model,
                          std::vector<Eigen::Vector3d> &plane_normals,
                          std::vector<double> &plane_d,
                          std::vector<double> &area,
                          std::vector<int> &duplicated_faces,
                          double min_area = 0.0) {
  for (int i = 0; i < faces_model.size(); i++) {
    if (std::find(duplicated_faces.begin(), duplicated_faces.end(), i) !=
            duplicated_faces.end() ||
        area.at(i) < min_area) {
      continue;
    }
    Eigen::Vector3d plane_normal_1 = plane_normals.at(i);
    double d_1 = plane_d.at(i);
    for (int j = i + 1; j < faces_model.size(); j++) {
      if (std::find(duplicated_faces.begin(), duplicated_faces.end(), j) !=
          duplicated_faces.end()) {
        continue;
      }
      Eigen::Vector3d plane_normal_2 = plane_normals.at(j);
      double d_2 = plane_d.at(j);

      if (std::fabs(plane_normal_1.dot(plane_normal_2)) > 0.9999 &&
          std::fabs(d_1 - d_2) < 0.003) {
        duplicated_faces.push_back(j);
      }
    }
  }
}

void processElementCloud(int idx,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr element_points,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr corners,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr corners_top_bottom,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr corners_side,
                         Eigen::Vector3d &element_normal, double &element_d) {
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.03);
  seg.setMaxIterations(1000);
  seg.setInputCloud(element_points);
  seg.segment(*inliers, *coefficients);

  // Compute ransac normal vector
  element_normal(0) = coefficients->values[0];
  element_normal(1) = coefficients->values[1],
  element_normal(2) = coefficients->values[2];
  element_normal.normalize();

  if (std::fabs(element_normal.z()) < 0.1) {
    element_normal.z() = 0;
    element_normal.normalize();
  }

  // TODO: ROS, get robot postion
  pcl::PointXYZ robot_position;
  if (idx == 4) {
    robot_position = pcl::PointXYZ(-9.4317884, -5.2247009, 1.0);
  } else {
    robot_position = pcl::PointXYZ(10.43749, 1.7694572, 1.0);
  }
  pcl::PointXYZ mean_point(0, 0, 0);
  for (int i = 0; i < element_points->size(); i++) {
    mean_point.x += (*element_points)[i].x;
    mean_point.y += (*element_points)[i].y;
    mean_point.z += (*element_points)[i].z;
  }
  mean_point.x /= element_points->size();
  mean_point.y /= element_points->size();
  mean_point.z /= element_points->size();
  Eigen::Vector3d shape_robot_vec(mean_point.x - robot_position.x,
                                  mean_point.y - robot_position.y,
                                  mean_point.z - robot_position.z);
  shape_robot_vec.normalize();
  // Normal direction of shape is pointing into free direction
  if (shape_robot_vec.dot(element_normal) < 0.0) {
    element_normal *= -1.;
  }
  element_d =
      -(mean_point.x * element_normal.x() + mean_point.y * element_normal.y() +
        mean_point.z * element_normal.z());

  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  searchTree->setInputCloud(element_points);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
  normalEstimator.setInputCloud(element_points);
  normalEstimator.setSearchMethod(searchTree);
  normalEstimator.setKSearch(10);
  normalEstimator.compute(*normals);

  // Filter corner points out
  pcl::PointCloud<pcl::PointXYZI>::Ptr corners_intensity(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal> harris;
  harris.setInputCloud(element_points);
  harris.setNonMaxSupression(false);
  harris.setRadius(0.2f);
  harris.setThreshold(0.2f);
  harris.setNormals(normals);
  harris.compute(*corners_intensity);

  // Remove Intensity field
  pcl::copyPointCloud(*corners_intensity, *corners);

  // Corners -> Corners top and bottom, Corners side
  if (std::fabs(element_normal.z()) < 0.1) {
    double max_z = -10000;
    double min_z = 10000;
    double max_xy = -10000;
    double min_xy = 10000;
    for (int i = 0; i < corners->size(); i++) {
      pcl::PointXYZ p = (*corners)[i];
      if (p.z > max_z) {
        max_z = p.z;
      }
      if (p.z < min_z) {
        min_z = p.z;
      }
      double dist = std::fabs(p.x) + std::fabs(p.y);
      if (dist > max_xy) {
        max_xy = dist;
      }
      if (dist < min_xy) {
        min_xy = dist;
      }
    }
    double hight = max_z - min_z;
    double width = max_xy - min_xy;
    double threshold_top = max_z - 0.15 * hight;
    double threshold_bottom = min_z + 0.15 * hight;
    double threshold_right = max_xy - 0.15 * width;
    double threshold_left = min_xy + 0.15 * width;

    for (int i = 0; i < corners->size(); i++) {
      pcl::PointXYZ p = (*corners)[i];
      if (p.z > threshold_top || p.z < threshold_bottom) {
        corners_top_bottom->push_back(p);
      }
      double dist = std::fabs(p.x) + std::fabs(p.y);
      if (dist > threshold_right || dist < threshold_left) {
        corners_side->push_back(p);
      }
    }
  } else {
    pcl::copyPointCloud(*corners, *corners_side);
    pcl::copyPointCloud(*corners, *corners_top_bottom);
  }
}

void selectMainCandidateFaces(
    std::vector<::pcl::Vertices> &faces_model,
    pcl::PointCloud<pcl::PointXYZ>::Ptr vertices_model,
    pcl::PointCloud<pcl::PointXYZ>::Ptr corners,
    Eigen::Vector3d &element_normal, double element_d,
    std::vector<int> &duplicated_faces, std::vector<int> &candidate_faces_main,
    std::vector<Eigen::Vector3d> &plane_normals, std::vector<double> &area,
    std::vector<double> &plane_d, double min_area = 0.0,
    bool plane_flipping = true) {
  std::vector<Eigen::Vector3d> flipped_planes;

  // Add planes with same similar normal and in local scope
  int nr_faces = faces_model.size();  // Break cycle!!
  for (int i = 0; i < nr_faces; i++) {
    // Ignore duplicated planes or very small areas
    if (std::find(duplicated_faces.begin(), duplicated_faces.end(), i) !=
            duplicated_faces.end() ||
        area.at(i) < min_area) {
      continue;
    }

    Eigen::Vector3d candidate_normal = plane_normals.at(i);
    double candidate_d = plane_d.at(i);

    // TODO: Consider robot position?
    if (std::fabs(element_normal.dot(candidate_normal)) > 0.99) {
      std::vector<uint32_t> vertices = faces_model.at(i).vertices;

      double min_distance = 1000;
      for (int p_idx = 0; p_idx < corners->size(); p_idx++) {
        pcl::PointXYZ p = (*corners)[p_idx];
        double error =
            std::fabs(candidate_normal.x() * p.x + candidate_normal.y() * p.y +
                      candidate_normal.z() * p.z + candidate_d);
        if (error < min_distance) {
          min_distance = error;
        }
      }

      if (min_distance <= 0.5) {
        candidate_faces_main.push_back(i);

        // Do plane flipping
        pcl::PointXYZ p1 = (*vertices_model)[vertices.at(0)];
        pcl::PointXYZ p2 = (*vertices_model)[vertices.at(1)];
        pcl::PointXYZ p3 = (*vertices_model)[vertices.at(2)];
        Eigen::Vector3d v1(p1.x, p1.y, p1.z);
        Eigen::Vector3d v2(p2.x, p2.y, p2.z);
        Eigen::Vector3d v3(p3.x, p3.y, p3.z);
        Eigen::Vector3d n = (v1 - v2).cross(v3 - v2);
        n.normalize();

        Eigen::Vector3d v1_v2 = v1 - v2;
        double length =
            std::sqrt(v1_v2.x() * v1_v2.x() + v1_v2.y() * v1_v2.y());

        if (plane_flipping && std::fabs(n.z()) < 0.1 && length >= 0.1 &&
            length <= 0.5) {
          bool already_flipped = false;
          for (int k = 0; k < flipped_planes.size(); k++) {
            Eigen::Vector3d flipping_vector = flipped_planes.at(k);
            if ((flipping_vector - (length * n)).lpNorm<Eigen::Infinity>() <
                0.01) {
              already_flipped = true;
              break;
            }
          }
          if (!already_flipped) {
            std::cout << "Do plane flipping" << std::endl;
            Eigen::Vector3d p1_pos(v1 + n * length);
            Eigen::Vector3d p2_pos(v2 + n * length);
            Eigen::Vector3d p3_pos(v3 + n * length);

            Eigen::Vector3d p1_neg(v1 - n * length);
            Eigen::Vector3d p2_neg(v2 - n * length);
            Eigen::Vector3d p3_neg(v3 - n * length);

            int vertices_size = vertices_model->size();
            vertices_model->push_back(
                pcl::PointXYZ(p1_pos.x(), p1_pos.y(), p1_pos.z()));
            vertices_model->push_back(
                pcl::PointXYZ(p2_pos.x(), p2_pos.y(), p2_pos.z()));
            vertices_model->push_back(
                pcl::PointXYZ(p3_pos.x(), p3_pos.y(), p3_pos.z()));
            pcl::Vertices vertices_shift_pos;
            vertices_shift_pos.vertices.push_back(vertices_size);
            vertices_shift_pos.vertices.push_back(vertices_size + 1);
            vertices_shift_pos.vertices.push_back(vertices_size + 2);

            vertices_model->push_back(
                pcl::PointXYZ(p1_neg.x(), p1_neg.y(), p1_neg.z()));
            vertices_model->push_back(
                pcl::PointXYZ(p2_neg.x(), p2_neg.y(), p2_neg.z()));
            vertices_model->push_back(
                pcl::PointXYZ(p3_neg.x(), p3_neg.y(), p3_neg.z()));
            pcl::Vertices vertices_shift_neg;
            vertices_shift_neg.vertices.push_back(vertices_size + 3);
            vertices_shift_neg.vertices.push_back(vertices_size + 4);
            vertices_shift_neg.vertices.push_back(vertices_size + 5);
            flipped_planes.push_back(n * length);

            candidate_faces_main.push_back(faces_model.size());
            faces_model.push_back(vertices_shift_pos);
            candidate_faces_main.push_back(faces_model.size());
            faces_model.push_back(vertices_shift_neg);

            plane_normals.push_back(n);
            plane_normals.push_back(n);
            area.push_back(n.norm());
            area.push_back(n.norm());

            double d1 =
                -(n.x() * p1_pos.x() + n.y() * p1_pos.y() + n.z() * p1_pos.z());
            double d2 =
                -(n.x() * p1_neg.x() + n.y() * p1_neg.y() + n.z() * p1_neg.z());
            plane_d.push_back(d1);
            plane_d.push_back(d2);
          }
        }
      }
    }
  }
}

void selectOrthoCandidateFaces(
    std::vector<::pcl::Vertices> &faces_model,
    std::vector<int> &duplicated_faces,
    pcl::PointCloud<pcl::PointXYZ>::Ptr vertices_model,
    pcl::PointCloud<pcl::PointXYZ>::Ptr corners_side,
    pcl::PointCloud<pcl::PointXYZ>::Ptr corners_top_bottom,
    Eigen::Vector3d &element_normal,
    std::vector<Eigen::Vector3d> &plane_normals, std::vector<double> &area,
    std::vector<double> &plane_d,
    std::vector<int> &candidate_faces_ortho_horizontal,
    std::vector<int> &candidate_faces_ortho_vertical, double min_area = 0.0) {
  for (int i = 0; i < faces_model.size(); i++) {
    // Ignore duplicated planes or very small areas
    if (std::find(duplicated_faces.begin(), duplicated_faces.end(), i) !=
            duplicated_faces.end() ||
        area.at(i) < min_area) {
      continue;
    }
    Eigen::Vector3d candidate_normal = plane_normals.at(i);
    double candidate_d = plane_d.at(i);

    if (std::fabs(element_normal.dot(candidate_normal)) < 0.06) {
      std::vector<uint32_t> vertices = faces_model[i].vertices;
      double min_distance = 1000;
      if (std::fabs(candidate_normal.z()) < 0.15) {
        for (int p_idx = 0; p_idx < corners_side->size(); p_idx++) {
          pcl::PointXYZ p = (*corners_side)[p_idx];
          double error = std::fabs(candidate_normal.x() * p.x +
                                   candidate_normal.y() * p.y +
                                   candidate_normal.z() * p.z + candidate_d);
          if (error < min_distance) {
            min_distance = error;
          }
        }
        if (min_distance <= 0.4) {
          candidate_faces_ortho_vertical.push_back(i);
        }
      } else if (std::fabs(candidate_normal.z()) > 0.85) {
        for (int p_idx = 0; p_idx < corners_top_bottom->size(); p_idx++) {
          pcl::PointXYZ p = (*corners_top_bottom)[p_idx];
          double error = std::fabs(candidate_normal.x() * p.x +
                                   candidate_normal.y() * p.y +
                                   candidate_normal.z() * p.z + candidate_d);
          if (error < min_distance) {
            min_distance = error;
          }
        }
        if (min_distance <= 1.5) {
          candidate_faces_ortho_horizontal.push_back(i);
        }
      }
    }
  }
}

void computeArtificialVertices(
    std::vector<int> &candidate_faces_main,
    std::vector<int> &candidate_faces_ortho_horizontal,
    std::vector<int> &candidate_faces_ortho_vertical,
    std::vector<Eigen::Vector3d> &plane_normals, std::vector<double> &plane_d,
    Eigen::Vector3d &element_normal, double &element_d,
    pcl::PointCloud<pcl::PointXYZ>::Ptr artificial_vertices,
    std::vector<Eigen::Matrix3d> &shape_directions) {
  Eigen::Vector3f p1;
  Eigen::Vector4f plane_m1, plane_h1, plane_v1;

  for (int i = 0; i < candidate_faces_main.size(); i++) {
    planeFromIdx(plane_m1, candidate_faces_main.at(i), plane_normals, plane_d);

    for (int j = 0; j < candidate_faces_ortho_vertical.size(); j++) {
      planeFromIdx(plane_v1, candidate_faces_ortho_vertical.at(j),
                   plane_normals, plane_d);

      for (int k = 0; k < candidate_faces_ortho_horizontal.size(); k++) {
        planeFromIdx(plane_h1, candidate_faces_ortho_horizontal.at(k),
                     plane_normals, plane_d);

        pcl::threePlanesIntersection(plane_m1, plane_v1, plane_h1, p1);
        artificial_vertices->push_back(pcl::PointXYZ(p1.x(), p1.y(), p1.z()));
      }
    }
  }

  Eigen::Matrix3d shape_direction;
  shape_direction.setZero();
  Eigen::Vector3d n1_zero = plane_normals.at(candidate_faces_main.at(0));
  Eigen::Vector3d n2_zero =
      plane_normals.at(candidate_faces_ortho_vertical.at(0));
  Eigen::Vector3d n3_zero =
      plane_normals.at(candidate_faces_ortho_horizontal.at(0));
  for (int i = 0; i < candidate_faces_main.size(); i++) {
    Eigen::Vector3d n = plane_normals.at(candidate_faces_main.at(i));
    if (n.dot(n1_zero) > 0) {
      shape_direction.col(0) += n;
    } else {
      shape_direction.col(0) -= n;
    }
  }
  shape_direction.col(0).normalize();

  for (int i = 0; i < candidate_faces_ortho_vertical.size(); i++) {
    Eigen::Vector3d n = plane_normals.at(candidate_faces_ortho_vertical.at(i));
    if (n.dot(n2_zero) > 0) {
      shape_direction.col(1) += n;
    } else {
      shape_direction.col(1) -= n;
    }
  }
  shape_direction.col(1).normalize();

  for (int i = 0; i < candidate_faces_ortho_horizontal.size(); i++) {
    Eigen::Vector3d n =
        plane_normals.at(candidate_faces_ortho_horizontal.at(i));
    if (n.dot(n3_zero) > 0) {
      shape_direction.col(2) += n;
    } else {
      shape_direction.col(2) -= n;
    }
  }
  shape_direction.col(2).normalize();
  shape_directions.push_back(shape_direction);
}

void removeDuplicatedPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                            double eps = 10e-6) {
  // Source:
  // https://stackoverflow.com/questions/34481190/removing-duplicates-of-3d-points-in-a-vector-in-c/34481426
  std::sort(cloud->begin(), cloud->end(),
            [eps](pcl::PointXYZ p1, pcl::PointXYZ p2) -> bool {
              if (std::fabs(p1.x - p2.x) > eps)
                return p1.x > p2.x;
              else if (std::fabs(p1.y - p2.y) > eps)
                return p1.y > p2.y;
              else
                return p1.z > p2.z;
            });
  auto unique_end =
      std::unique(cloud->begin(), cloud->end(),
                  [eps](pcl::PointXYZ p1, pcl::PointXYZ p2) -> bool {
                    if (pcl::euclideanDistance(p1, p2) < eps) {
                      return true;
                    }
                    return false;
                  });
  cloud->erase(unique_end, cloud->end());
}

void removeDuplicatedValues(std::vector<double> &vector, double eps = 10e-6) {
  // Source:
  // https://stackoverflow.com/questions/34481190/removing-duplicates-of-3d-points-in-a-vector-in-c/34481426
  std::sort(vector.begin(), vector.end(),
            [eps](double p1, double p2) -> bool { return p1 > p2; });
  auto unique_end = std::unique(vector.begin(), vector.end(),
                                [eps](double p1, double p2) -> bool {
                                  if (std::fabs(p1 - p2) < eps) {
                                    return true;
                                  }
                                  return false;
                                });
  vector.erase(unique_end, vector.end());
}

bool getReconstructionParameters(
    int idx,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &strong_points_reconstruction,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &weak_points,
    std::vector<Eigen::Vector3d> normal_detected_shapes_vector,
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> element_points_vector,
    std::vector<pcl::search::KdTree<pcl::PointXYZ>::Ptr>
        element_points_kdtrees_vector,
    std::vector<Eigen::Vector3d> &center_estimates,
    std::vector<Eigen::Matrix3d> &direction_estimates,
    std::vector<std::vector<Eigen::VectorXd>> &parameter_estimates,
    Eigen::Matrix3d &averaged_normals) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr upsampled_model(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PLYReader reader;
  reader.read("/home/philipp/Schreibtisch/data/CLA_MissingParts_3_5m.ply",
              *upsampled_model);

  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(0.05f);
  octree.setInputCloud(upsampled_model);
  octree.addPointsFromInputCloud();

  pcl::search::KdTree<pcl::PointXYZ> upsampled_model_kdtree;
  upsampled_model_kdtree.setInputCloud(upsampled_model);

  std::vector<int> nn_indices{1};
  std::vector<float> nn_dists{1};

  // Get data
  pcl::PointCloud<pcl::PointXYZ>::Ptr element_points =
      element_points_vector.at(idx);
  Eigen::Vector3d element_normal = normal_detected_shapes_vector.at(idx);

  // TODO: ROS, get robot postion
  pcl::PointXYZ robot_position(10.43749, 1.7694572, 1.0);
  pcl::PointXYZ mean_point(0, 0, 0);
  for (int i = 0; i < element_points->size(); i++) {
    mean_point.x += (*element_points)[i].x;
    mean_point.y += (*element_points)[i].y;
    mean_point.z += (*element_points)[i].z;
  }
  mean_point.x /= element_points->size();
  mean_point.y /= element_points->size();
  mean_point.z /= element_points->size();
  Eigen::Vector3d shape_robot_vec(mean_point.x - robot_position.x,
                                  mean_point.y - robot_position.y,
                                  mean_point.z - robot_position.z);
  shape_robot_vec.normalize();
  // Normal direction of shape is pointing into free direction
  if (shape_robot_vec.dot(element_normal) < 0.0) {
    // element_normal *= -1.;
    std::cout << "element_normal pointing in wrong direction!" << std::endl;
    // assert(false);
  }

  double shape_d =
      -(mean_point.x * element_normal.x() + mean_point.y * element_normal.y() +
        mean_point.z * element_normal.z());

  // Filter points (remaining vertices -> vertices on axis and free side)
  // TODO: Maybe recompute plane from vertices to reduce the tolerance?
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = 0; i < weak_points->size(); i++) {
    pcl::PointXYZ p = (*weak_points)[i];
    if (p.x * element_normal.x() + p.y * element_normal.y() +
            p.z * element_normal.z() + shape_d >=
        -0.05) {
      filtered_cloud->push_back(p);
    }
  }

  // Corp off confliction points
  // Source: https://pcl.readthedocs.io/en/latest/moment_of_inertia.html
  pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud(filtered_cloud);
  feature_extractor.compute();
  Eigen::Vector3f major_vector, middle_vector, minor_vector;
  Eigen::Vector3f mass_center;
  feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
  // feature_extractor.getMassCenter(mass_center);
  Eigen::Vector3f mean_e(mean_point.x, mean_point.y, mean_point.z);
  mass_center = mean_e + 0.1 * element_normal.cast<float>();

  Eigen::Matrix3d corrected_dir = averaged_normals;

  direction_estimates.push_back(corrected_dir);

  Eigen::Matrix<float, 3, 6> shooting_dir;
  shooting_dir.col(0) = corrected_dir.col(0).cast<float>();
  shooting_dir.col(1) = -corrected_dir.col(0).cast<float>();
  shooting_dir.col(2) = corrected_dir.col(1).cast<float>();
  shooting_dir.col(3) = -corrected_dir.col(1).cast<float>();
  shooting_dir.col(4) = corrected_dir.col(2).cast<float>();
  shooting_dir.col(5) = -corrected_dir.col(2).cast<float>();

  std::cout << shooting_dir << std::endl;
  for (int i = 0; i < 6; i++) {
    pcl::PointCloud<pcl::PointXYZ>::VectorType voxel_centers;
    Eigen::Vector3f direction = shooting_dir.col(i);
    octree.getIntersectedVoxelCenters(mass_center, direction, voxel_centers);

    if (voxel_centers.size() > 0) {
      pcl::PointXYZ voxel_center = voxel_centers.at(0);

      upsampled_model_kdtree.nearestKSearch(voxel_center, 1, nn_indices,
                                            nn_dists);
      pcl::PointXYZ boundary_point = (*upsampled_model)[nn_indices.at(0)];

      double d_boundary = -(direction.x() * boundary_point.x +
                            direction.y() * boundary_point.y +
                            direction.z() * boundary_point.z);

      pcl::PointIndices::Ptr indices_to_remove(new pcl::PointIndices());
      for (int j = 0; j < filtered_cloud->size(); j++) {
        pcl::PointXYZ p = (*filtered_cloud)[j];
        double error = p.x * direction.x() + p.y * direction.y() +
                       p.z * direction.z() + d_boundary;
        if (error > 0.04) {
          indices_to_remove->indices.push_back(j);
        }
      }
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud(filtered_cloud);
      extract.setIndices(indices_to_remove);
      extract.setNegative(true);
      extract.filter(*filtered_cloud);
    }
  }

  // feature_extractor.setInputCloud(filtered_cloud);
  // feature_extractor.compute();
  // feature_extractor.getMassCenter(mass_center);
  center_estimates.push_back(mass_center.cast<double>());

  removeDuplicatedPoints(filtered_cloud, 0.003);

  std::string save2 = "/home/philipp/Schreibtisch/resulting_points" +
                      std::to_string(idx) + ".ply";
  pcl::io::savePLYFile(save2, *filtered_cloud);

  // Structure of strong points
  pcl::PointCloud<pcl::PointXYZ>::Ptr alive_strong_points_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  for (int i = 0; i < strong_points_reconstruction->size(); i++) {
    pcl::PointXYZ p_f = (*strong_points_reconstruction)[i];
    Eigen::Vector3d e_f(p_f.x, p_f.y, p_f.z);
    for (int j = 0; j < filtered_cloud->size(); j++) {
      pcl::PointXYZ p_s = (*filtered_cloud)[j];
      Eigen::Vector3d e_s(p_s.x, p_s.y, p_s.z);
      double error = (e_f - e_s).lpNorm<Eigen::Infinity>();
      if (error < 0.05) {
        alive_strong_points_cloud->push_back(p_f);
        break;
      }
    }
  }
  removeDuplicatedPoints(alive_strong_points_cloud);

  std::string save3 = "/home/philipp/Schreibtisch/alive_strong_points_cloud" +
                      std::to_string(idx) + ".ply";
  pcl::io::savePLYFile(save3, *alive_strong_points_cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr alive_element_points(
      new pcl::PointCloud<pcl::PointXYZ>());
  std::vector<Eigen::Vector3d> alive_element_points_normal;
  for (int i = 0; i < element_points_vector.size(); i++) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cur_element_points =
        element_points_vector.at(i);
    Eigen::Vector3d cur_element_normal = normal_detected_shapes_vector.at(i);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr element_kdtree =
        element_points_kdtrees_vector.at(i);
    for (int j = 0; j < filtered_cloud->size(); j++) {
      pcl::PointXYZ p_s = (*filtered_cloud)[j];
      element_kdtree->nearestKSearch(p_s, 1, nn_indices, nn_dists);
      if (nn_dists[0] < 0.01) {
        alive_element_points->push_back(p_s);
        alive_element_points_normal.push_back(cur_element_normal);
      }
    }
  }

  std::cout << "Nr. Strong Point alive: " << alive_strong_points_cloud->size()
            << std::endl;
  std::cout << "Nr. Element Points alive: " << alive_element_points->size()
            << std::endl;
  // a1, a2, b1, b2, c1, c2
  // a -> direction of major_vector
  // b -> direction of middle_vector
  // c -> direction of minor_vector
  std::vector<double> a1, a2, b1, b2, c1, c2;

  for (int i = 0; i < alive_strong_points_cloud->size(); i++) {
    pcl::PointXYZ cur_point = (*alive_strong_points_cloud)[i];
    Eigen::Vector3f e_p(cur_point.x, cur_point.y, cur_point.z);
    Eigen::Vector3f center_point = e_p - mass_center;
    double a = center_point.dot(corrected_dir.col(0).cast<float>());
    double b = center_point.dot(corrected_dir.col(1).cast<float>());
    double c = center_point.dot(corrected_dir.col(2).cast<float>());
    if (a > 0) {
      a1.push_back(a);
    } else {
      a2.push_back(a);
    }
    if (b > 0) {
      b1.push_back(b);
    } else {
      b2.push_back(b);
    }
    if (c > 0) {
      c1.push_back(c);
    } else {
      c2.push_back(c);
    }
  }

  std::cout << "a1: " << a1.size() << std::endl;
  std::cout << "a2: " << a2.size() << std::endl;
  std::cout << "b1: " << b1.size() << std::endl;
  std::cout << "b2: " << b2.size() << std::endl;
  std::cout << "c1: " << c1.size() << std::endl;
  std::cout << "c2: " << c2.size() << std::endl;

  bool a1_incomplete = a1.empty();
  bool a2_incomplete = a2.empty();
  bool b1_incomplete = b1.empty();
  bool b2_incomplete = b2.empty();
  bool c1_incomplete = c1.empty();
  bool c2_incomplete = c2.empty();

  for (int i = 0; i < alive_element_points->size(); i++) {
    pcl::PointXYZ cur_point = (*alive_element_points)[i];
    Eigen::Vector3f e_p(cur_point.x, cur_point.y, cur_point.z);
    Eigen::Vector3f center_point = e_p - mass_center;
    double a = center_point.dot(corrected_dir.col(0).cast<float>());
    double b = center_point.dot(corrected_dir.col(1).cast<float>());
    double c = center_point.dot(corrected_dir.col(2).cast<float>());

    Eigen::Vector3d cur_element_normal = alive_element_points_normal.at(i);
    bool ortho_a =
        std::fabs(cur_element_normal.dot(corrected_dir.col(0))) > 0.9;
    bool ortho_b =
        std::fabs(cur_element_normal.dot(corrected_dir.col(1))) > 0.9;
    bool ortho_c =
        std::fabs(cur_element_normal.dot(corrected_dir.col(2))) > 0.9;

    if (ortho_a) {
      if (a > 0 && a1_incomplete) {
        a1.push_back(a);
      } else if (a < 0 && a2_incomplete) {
        a2.push_back(a);
      }
    }

    if (ortho_b) {
      if (b > 0 && b1_incomplete) {
        b1.push_back(b);
      } else if (b < 0 && b2_incomplete) {
        b2.push_back(b);
      }
    }
    if (ortho_c) {
      if (c > 0 && c1_incomplete) {
        c1.push_back(c);
      } else if (c < 0 && c2_incomplete) {
        c2.push_back(c);
      }
    }
  }

  std::cout << "a1: " << a1.size() << std::endl;
  std::cout << "a2: " << a2.size() << std::endl;
  std::cout << "b1: " << b1.size() << std::endl;
  std::cout << "b2: " << b2.size() << std::endl;
  std::cout << "c1: " << c1.size() << std::endl;
  std::cout << "c2: " << c2.size() << std::endl;

  a1_incomplete = a1.empty();
  a2_incomplete = a2.empty();
  b1_incomplete = b1.empty();
  b2_incomplete = b2.empty();
  c1_incomplete = c1.empty();
  c2_incomplete = c2.empty();

  for (int i = 0; i < filtered_cloud->size(); i++) {
    pcl::PointXYZ cur_point = (*filtered_cloud)[i];
    Eigen::Vector3f e_p(cur_point.x, cur_point.y, cur_point.z);
    Eigen::Vector3f center_point = e_p - mass_center;
    double a = center_point.dot(corrected_dir.col(0).cast<float>());
    double b = center_point.dot(corrected_dir.col(1).cast<float>());
    double c = center_point.dot(corrected_dir.col(2).cast<float>());
    if (a > 0 && a1_incomplete) {
      a1.push_back(a);
    } else if (a < 0 && a2_incomplete) {
      a2.push_back(a);
    }

    if (b > 0 && b1_incomplete) {
      b1.push_back(b);
    } else if (b < 0 && b2_incomplete) {
      b2.push_back(b);
    }

    if (c > 0 && c1_incomplete) {
      c1.push_back(c);
    } else if (c < 0 && c2_incomplete) {
      c2.push_back(c);
    }
  }

  std::cout << "a1: " << a1.size() << std::endl;
  std::cout << "a2: " << a2.size() << std::endl;
  std::cout << "b1: " << b1.size() << std::endl;
  std::cout << "b2: " << b2.size() << std::endl;
  std::cout << "c1: " << c1.size() << std::endl;
  std::cout << "c2: " << c2.size() << std::endl;

  removeDuplicatedValues(a1, 0.03);
  removeDuplicatedValues(a2, 0.03);
  removeDuplicatedValues(b1, 0.03);
  removeDuplicatedValues(b2, 0.03);
  removeDuplicatedValues(c1, 0.03);
  removeDuplicatedValues(c2, 0.03);

  std::cout << "a1: " << a1.size() << std::endl;
  std::cout << "a2: " << a2.size() << std::endl;
  std::cout << "b1: " << b1.size() << std::endl;
  std::cout << "b2: " << b2.size() << std::endl;
  std::cout << "c1: " << c1.size() << std::endl;
  std::cout << "c2: " << c2.size() << std::endl;

  Eigen::VectorXd a1_vec =
      Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(a1.data(), a1.size());
  Eigen::VectorXd a2_vec =
      Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(a2.data(), a2.size());
  Eigen::VectorXd b1_vec =
      Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(b1.data(), b1.size());
  Eigen::VectorXd b2_vec =
      Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(b2.data(), b2.size());
  Eigen::VectorXd c1_vec =
      Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(c1.data(), c1.size());
  Eigen::VectorXd c2_vec =
      Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(c2.data(), c2.size());

  std::vector<Eigen::VectorXd> parameters;
  parameters.push_back(a1_vec);
  parameters.push_back(a2_vec);
  parameters.push_back(b1_vec);
  parameters.push_back(b2_vec);
  parameters.push_back(c1_vec);
  parameters.push_back(c2_vec);
  parameter_estimates.push_back(parameters);

  return true;
}

int main() {
  double min_area = 0.0;

  pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFilePLY(
      "/home/philipp/Schreibtisch/data/CLA_MissingParts_3.ply", mesh);

  pcl::PolygonMesh mesh_detected;
  pcl::io::loadPolygonFilePLY(
      "/home/philipp/Schreibtisch/Result_CGAL/mesh_all_10.ply", mesh_detected);

  pcl::PointCloud<pcl::PointXYZ>::Ptr vertices_model_only(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(mesh.cloud, *vertices_model_only);
  std::vector<::pcl::Vertices> faces_model_only = mesh.polygons;

  combineMeshes(mesh_detected, mesh);

  std::vector<::pcl::Vertices> faces_model = mesh.polygons;
  pcl::PointCloud<pcl::PointXYZ>::Ptr vertices_model(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(mesh.cloud, *vertices_model);

  // Get all plane parameters from model
  std::vector<double> plane_d;
  std::vector<double> area;
  std::vector<Eigen::Vector3d> plane_normals;
  computeAllPlanes(faces_model, vertices_model, plane_normals, plane_d, area,
                   true, 0.1);

  // Flag duplicated planes
  std::vector<int> duplicated_faces;
  flagDuplicatedPlanes(faces_model, plane_normals, plane_d, area,
                       duplicated_faces, min_area);

  // Compute artificial Vertices for all shapes
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> artificial_vertices_vector;
  std::vector<pcl::search::KdTree<pcl::PointXYZ>::Ptr>
      artificial_kdtrees_vector;
  std::vector<Eigen::Vector3d> normal_detected_shapes_vector;
  std::vector<double> detected_shapes_d_vector;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> element_points_vector;
  std::vector<pcl::search::KdTree<pcl::PointXYZ>::Ptr>
      element_points_kdtrees_vector;
  std::vector<int> shapes;
  shapes.push_back(0);
  shapes.push_back(1);
  shapes.push_back(2);
  shapes.push_back(3);
  shapes.push_back(4);
  shapes.push_back(10);
  shapes.push_back(28);
  shapes.push_back(30);
  shapes.push_back(49);
  shapes.push_back(56);
  shapes.push_back(66);

  std::vector<Eigen::Matrix3d> shape_directions;
  std::string path = "/home/philipp/Schreibtisch/Result_CGAL/Meshes/";
  for (int i = 0; i < shapes.size(); i++) {
    std::string filename = "points_" + std::to_string(shapes.at(i)) + ".ply";
    std::string path_filename = path + filename;

    // Load element point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr element_points(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPLYFile<pcl::PointXYZ>(path_filename, *element_points);
    // Normal of detection
    pcl::PointCloud<pcl::PointXYZ>::Ptr corners(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr corners_top_bottom(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr corners_side(
        new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Vector3d element_normal;
    double element_d;
    processElementCloud(i, element_points, corners, corners_top_bottom,
                        corners_side, element_normal, element_d);
    normal_detected_shapes_vector.push_back(element_normal);
    detected_shapes_d_vector.push_back(element_d);
    element_points_vector.push_back(corners);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr element_kd_tree(
        new pcl::search::KdTree<pcl::PointXYZ>());
    element_kd_tree->setInputCloud(element_points);
    element_points_kdtrees_vector.push_back(element_kd_tree);

    // Candidates: local scope
    std::vector<int> candidate_faces_main;
    selectMainCandidateFaces(faces_model, vertices_model, corners,
                             element_normal, element_d, duplicated_faces,
                             candidate_faces_main, plane_normals, area, plane_d,
                             min_area, true);

    // Add planes with orthogonal normal and in local scope
    std::vector<int> candidate_faces_ortho_horizontal;
    std::vector<int> candidate_faces_ortho_vertical;
    selectOrthoCandidateFaces(faces_model, duplicated_faces, vertices_model,
                              corners_side, corners_top_bottom, element_normal,
                              plane_normals, area, plane_d,
                              candidate_faces_ortho_horizontal,
                              candidate_faces_ortho_vertical, min_area);

    int nr_main = candidate_faces_main.size();
    int nr_ortho_vertical = candidate_faces_ortho_vertical.size();
    int nr_ortho_horizontal = candidate_faces_ortho_horizontal.size();
    std::cout << "Number of main: " << nr_main << std::endl;
    std::cout << "Number of ortho_vertical: " << nr_ortho_vertical << std::endl;
    std::cout << "Number of ortho_horizontal: " << nr_ortho_horizontal
              << std::endl;

    // Compute all intersections directly
    pcl::PointCloud<pcl::PointXYZ>::Ptr artificial_vertices(
        new pcl::PointCloud<pcl::PointXYZ>);
    computeArtificialVertices(
        candidate_faces_main, candidate_faces_ortho_horizontal,
        candidate_faces_ortho_vertical, plane_normals, plane_d, element_normal,
        element_d, artificial_vertices, shape_directions);

    artificial_vertices_vector.push_back(artificial_vertices);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree(
        new pcl::search::KdTree<pcl::PointXYZ>());
    kd_tree->setInputCloud(artificial_vertices);
    artificial_kdtrees_vector.push_back(kd_tree);
  }

  std::cout << "element params:" << std::endl;
  for (int i = 0; i < shape_directions.size(); i++) {
    std::cout << shape_directions.at(i) << std::endl;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr all_verticies(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = 0; i < artificial_vertices_vector.size(); i++) {
    (*all_verticies) += *(artificial_vertices_vector.at(i));
  }
  pcl::io::savePLYFile("/home/philipp/Schreibtisch/all_artificial_vertices.ply",
                       *all_verticies);

  // Split point cloud in three parts:
  // 1. Points which are matching the vertices in the building model (strong
  // points)
  // 2. Points computed multiple times (weak points)
  // 3. Remaining points (backup)
  pcl::PointCloud<pcl::PointXYZ>::Ptr strong_points(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr weak_points(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr backup_points(
      new pcl::PointCloud<pcl::PointXYZ>);

  std::vector<int> nn_indices{1};
  std::vector<float> nn_dists{1};

  for (int i = 0; i < vertices_model_only->size(); i++) {
    pcl::PointXYZ p_model = (*vertices_model_only)[i];
    for (int j = 0; j < artificial_kdtrees_vector.size(); j++) {
      artificial_kdtrees_vector.at(j)->nearestKSearch(p_model, 1, nn_indices,
                                                      nn_dists);
      if (nn_dists[0] < 10e-3) {
        strong_points->push_back(p_model);
        break;
      }
    }
  }

  for (int i1 = 0; i1 < artificial_vertices_vector.size(); i1++) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cur_cloud =
        artificial_vertices_vector.at(i1);
    for (int i2 = 0; i2 < cur_cloud->size(); i2++) {
      pcl::PointXYZ cur_point = (*cur_cloud)[i2];
      for (int j = 0; j < artificial_kdtrees_vector.size(); j++) {
        if (i1 == j) {
          continue;
        }
        pcl::search::KdTree<pcl::PointXYZ>::Ptr cur_kd_tree =
            artificial_kdtrees_vector.at(j);
        cur_kd_tree->nearestKSearch(cur_point, 1, nn_indices, nn_dists);
        if (nn_dists[0] < 10e-10) {
          weak_points->push_back(cur_point);
          break;
        }
      }
    }
  }

  // Remove duplicates
  removeDuplicatedPoints(strong_points);
  removeDuplicatedPoints(weak_points);

  pcl::io::savePLYFile("/home/philipp/Schreibtisch/strong_points.ply",
                       *strong_points);
  pcl::io::savePLYFile("/home/philipp/Schreibtisch/weak_points.ply",
                       *weak_points);

  // reconstruction
  // Starting with elements element with the highest support of existing
  // vertices
  int max_idx = 0;
  int max_value = 0;
  pcl::PolygonMesh mesh_all;
  std::vector<int> done_shapes;
  pcl::PointCloud<pcl::PointXYZ>::Ptr strong_points_reconstruction;
  std::vector<Eigen::Vector3d> center_estimates;
  std::vector<Eigen::Matrix3d> direction_estimates;
  std::vector<std::vector<Eigen::VectorXd>> parameter_estimates;

  do {
    for (int i = 0; i < artificial_kdtrees_vector.size(); i++) {
      if (std::find(done_shapes.begin(), done_shapes.end(), i) !=
          done_shapes.end()) {
        continue;
      }

      int counter_model_vertices = 0;
      pcl::PointCloud<pcl::PointXYZ>::Ptr strong_points_reconstruction_temp(
          new pcl::PointCloud<pcl::PointXYZ>);
      for (int j = 0; j < strong_points->size(); j++) {
        pcl::PointXYZ p = (*strong_points)[j];
        artificial_kdtrees_vector.at(i)->nearestKSearch(p, 1, nn_indices,
                                                        nn_dists);
        if (nn_dists[0] < 10e-4) {
          counter_model_vertices++;
          strong_points_reconstruction_temp->push_back(p);
        }
      }
      if (max_value <= counter_model_vertices) {
        max_value = counter_model_vertices;
        max_idx = i;
        strong_points_reconstruction = strong_points_reconstruction_temp;
      }
    }

    // Reconstruct i and remove it
    std::cout << "Element to reconstruct: " << max_idx
              << " number of strong points: " << max_value << std::endl;

    getReconstructionParameters(
        max_idx, strong_points_reconstruction,
        artificial_vertices_vector.at(max_idx), normal_detected_shapes_vector,
        element_points_vector, element_points_kdtrees_vector, center_estimates,
        direction_estimates, parameter_estimates, shape_directions.at(max_idx));

    pcl::PointCloud<pcl::PointXYZ>::Ptr new_strong_points(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromPCLPointCloud2(mesh_all.cloud, *new_strong_points);
    *strong_points += *new_strong_points;
    done_shapes.push_back(max_idx);
    max_idx = 0;
    max_value = 0;
  } while (done_shapes.size() < artificial_vertices_vector.size());

  for (int i = 0; i < center_estimates.size(); i++) {
    Eigen::Vector3d center_estimates_0 = center_estimates.at(i);
    Eigen::Matrix3d direction_estimates_0 = direction_estimates.at(i);
    std::vector<Eigen::VectorXd> parameter_estimates_0 =
        parameter_estimates.at(i);

    Eigen::VectorXd a1 = parameter_estimates_0.at(0);
    Eigen::VectorXd a2 = parameter_estimates_0.at(1);
    Eigen::VectorXd b1 = parameter_estimates_0.at(2);
    Eigen::VectorXd b2 = parameter_estimates_0.at(3);
    Eigen::VectorXd c1 = parameter_estimates_0.at(4);
    Eigen::VectorXd c2 = parameter_estimates_0.at(5);

    std::cout << a1.size() << std::endl;
    std::cout << a2.size() << std::endl;
    std::cout << b1.size() << std::endl;
    std::cout << b2.size() << std::endl;
    std::cout << c1.size() << std::endl;
    std::cout << c2.size() << std::endl;

    if (a1.size() == 0 || a2.size() == 0 || b1.size() == 0 || b2.size() == 0 ||
        c1.size() == 0 || c2.size() == 0) {
      std::cout << "Skipped element " << i << std::endl;
      continue;
    }

    double a1_max = a1.maxCoeff() > std::fabs(a1.minCoeff()) ? a1.maxCoeff()
                                                             : a1.minCoeff();
    double a2_max = a2.maxCoeff() > std::fabs(a2.minCoeff()) ? a2.maxCoeff()
                                                             : a2.minCoeff();
    double b1_max = b1.maxCoeff() > std::fabs(b1.minCoeff()) ? b1.maxCoeff()
                                                             : b1.minCoeff();
    double b2_max = b2.maxCoeff() > std::fabs(b2.minCoeff()) ? b2.maxCoeff()
                                                             : b2.minCoeff();
    double c1_max = c1.maxCoeff() > std::fabs(c1.minCoeff()) ? c1.maxCoeff()
                                                             : c1.minCoeff();
    double c2_max = c2.maxCoeff() > std::fabs(c2.minCoeff()) ? c2.maxCoeff()
                                                             : c2.minCoeff();

    std::cout << "a1_max" << a1_max << std::endl;
    std::cout << "a2_max" << a2_max << std::endl;
    std::cout << "b1_max" << b1_max << std::endl;
    std::cout << "b2_max" << b2_max << std::endl;
    std::cout << "c1_max" << c1_max << std::endl;
    std::cout << "c2_max" << c2_max << std::endl;

    Eigen::Vector3d p1 = center_estimates_0 +
                         direction_estimates_0.col(0) * a1_max +
                         direction_estimates_0.col(1) * b1_max +
                         direction_estimates_0.col(2) * c1_max;

    Eigen::Vector3d p2 = center_estimates_0 +
                         direction_estimates_0.col(0) * a1_max +
                         direction_estimates_0.col(1) * b1_max +
                         direction_estimates_0.col(2) * c2_max;

    Eigen::Vector3d p3 = center_estimates_0 +
                         direction_estimates_0.col(0) * a1_max +
                         direction_estimates_0.col(1) * b2_max +
                         direction_estimates_0.col(2) * c1_max;

    Eigen::Vector3d p4 = center_estimates_0 +
                         direction_estimates_0.col(0) * a1_max +
                         direction_estimates_0.col(1) * b2_max +
                         direction_estimates_0.col(2) * c2_max;

    Eigen::Vector3d p5 = center_estimates_0 +
                         direction_estimates_0.col(0) * a2_max +
                         direction_estimates_0.col(1) * b1_max +
                         direction_estimates_0.col(2) * c1_max;

    Eigen::Vector3d p6 = center_estimates_0 +
                         direction_estimates_0.col(0) * a2_max +
                         direction_estimates_0.col(1) * b1_max +
                         direction_estimates_0.col(2) * c2_max;

    Eigen::Vector3d p7 = center_estimates_0 +
                         direction_estimates_0.col(0) * a2_max +
                         direction_estimates_0.col(1) * b2_max +
                         direction_estimates_0.col(2) * c1_max;

    Eigen::Vector3d p8 = center_estimates_0 +
                         direction_estimates_0.col(0) * a2_max +
                         direction_estimates_0.col(1) * b2_max +
                         direction_estimates_0.col(2) * c2_max;

    pcl::PointCloud<pcl::PointXYZ>::Ptr element(
        new pcl::PointCloud<pcl::PointXYZ>());
    element->push_back(pcl::PointXYZ(p1.x(), p1.y(), p1.z()));
    element->push_back(pcl::PointXYZ(p2.x(), p2.y(), p2.z()));
    element->push_back(pcl::PointXYZ(p3.x(), p3.y(), p3.z()));
    element->push_back(pcl::PointXYZ(p4.x(), p4.y(), p4.z()));
    element->push_back(pcl::PointXYZ(p5.x(), p5.y(), p5.z()));
    element->push_back(pcl::PointXYZ(p6.x(), p6.y(), p6.z()));
    element->push_back(pcl::PointXYZ(p7.x(), p7.y(), p7.z()));
    element->push_back(pcl::PointXYZ(p8.x(), p8.y(), p8.z()));

    std::string save = "/home/philipp/Schreibtisch/final_meshing_points" +
                       std::to_string(i) + ".ply";
    pcl::io::savePLYFile(save, *element);

    try {
      pcl::PolygonMesh reconstructed_mesh;
      pcl::ConvexHull<pcl::PointXYZ> chull;
      chull.setInputCloud(element);
      chull.setDimension(3);
      chull.reconstruct(reconstructed_mesh);

      combineMeshes(reconstructed_mesh, mesh_all);
    } catch (...) {
      std::cout << " Couldn't reconstruct convex hull" << std::endl;
    }
  }
  pcl::io::savePLYFile("/home/philipp/Schreibtisch/reconstructed_mesh_el_0.ply",
                       mesh_all);

  return 0;
}