#include <cmath>
#include <iostream>
#include <utility>

#include <Eigen/Core>

#include <pcl/ModelCoefficients.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
//#include <pcl/common/impl/intersections.hpp>
#include <pcl/common/intersections.h>

#include <pcl/PCLHeader.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PolygonMesh.h>
#include <pcl/Vertices.h>
#include <pcl/common/distances.h>
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

void combineMeshes(const pcl::PolygonMesh &mesh, pcl::PolygonMesh &mesh_all) {
  // pcl::PolygonMesh::concatenate(mesh_all, mesh); ???
  // Source:
  // https://github.com/PointCloudLibrary/pcl/blob/master/common/include/pcl/PolygonMesh.h
  mesh_all.header.stamp = std::max(mesh_all.header.stamp, mesh.header.stamp);

  const auto point_offset = mesh_all.cloud.width * mesh_all.cloud.height;

  pcl::PCLPointCloud2 new_cloud;
  pcl::concatenatePointCloud(mesh_all.cloud, mesh.cloud, new_cloud);
  mesh_all.cloud = new_cloud;

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

int main() {
  // TODO: include all detected planes for candidate generation!

  pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFilePLY(
      "/home/philipp/Schreibtisch/data/CLA_MissingParts_1.ply", mesh);

  // Load element point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr element_points(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPLYFile<pcl::PointXYZ>(
      "/home/philipp/Schreibtisch/Result_CGAL/Meshes/points_2.ply",
      *element_points);

  std::vector<::pcl::Vertices> faces_model = mesh.polygons;
  pcl::PointCloud<pcl::PointXYZ>::Ptr vertices_model(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(mesh.cloud, *vertices_model);

  // Get all plane parameters from model
  std::vector<double> plane_d;
  std::vector<double> area;
  std::vector<Eigen::Vector3d> plane_normals;
  for (int i = 0; i < faces_model.size(); i++) {
    std::vector<uint32_t> vertices = faces_model.at(i).vertices;
    pcl::PointXYZ p1 = (*vertices_model)[vertices.at(0)];
    pcl::PointXYZ p2 = (*vertices_model)[vertices.at(1)];
    pcl::PointXYZ p3 = (*vertices_model)[vertices.at(2)];
    Eigen::Vector3d v1(p1.x, p1.y, p1.z);
    Eigen::Vector3d v2(p2.x, p2.y, p2.z);
    Eigen::Vector3d v3(p3.x, p3.y, p3.z);
    Eigen::Vector3d n = (v1 - v2).cross(v3 - v2);
    area.push_back(n.norm());
    n.normalize();

    plane_normals.push_back(n);
    double d = -(n.x() * p1.x + n.y() * p1.y + n.z() * p1.z);
    plane_d.push_back(d);
  }

  // Flag duplicated planes
  std::vector<int> duplicated_faces;
  for (int i = 0; i < faces_model.size(); i++) {
    if (std::find(duplicated_faces.begin(), duplicated_faces.end(), i) !=
        duplicated_faces.end()) {
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

      if (std::abs(plane_normal_1.dot(plane_normal_2)) > 0.999 &&
          std::abs(d_1 - d_2) < 0.0001) {
        duplicated_faces.push_back(j);
      }
    }
  }

  // Normal of detection
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

  Eigen::Vector3d element_normal(coefficients->values[0],
                                 coefficients->values[1],
                                 coefficients->values[2]);
  element_normal.normalize();
  pcl::PointXYZ test_point = (*element_points)[0];
  double element_d =
      -(element_normal.x() * test_point.x + element_normal.y() * test_point.y +
        element_normal.z() * test_point.z);

  // Candidates: local scope
  std::vector<int> candidate_faces_main;
  // Add planes with same similar normal and in local scope
  int nr_faces = faces_model.size();  // Break cycle!!
  for (int i = 0; i < nr_faces; i++) {
    // Ignore duplicated planes or very small areas
    if (std::find(duplicated_faces.begin(), duplicated_faces.end(), i) !=
            duplicated_faces.end() ||
        area.at(i) < 0.01) {
      continue;
    }

    Eigen::Vector3d candidate_normal = plane_normals.at(i);

    if (std::abs(element_normal.dot(candidate_normal)) > 0.9) {
      std::vector<uint32_t> vertices = faces_model.at(i).vertices;
      double sum_distance = 0;
      for (int v_idx = 0; v_idx < 3; v_idx++) {
        pcl::PointXYZ p = (*vertices_model)[vertices.at(v_idx)];
        double error =
            std::abs(element_normal.x() * p.x + element_normal.y() * p.y +
                     element_normal.z() * p.z + element_d);
        sum_distance += error;
      }
      double mean_distance = sum_distance / 3.0;
      if (mean_distance <= 0.5) {
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

        if (std::abs(n.z()) < 0.1 && length >= 0.1 && length <= 0.5) {
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

  // Add planes with orthogonal normal and in local scope
  std::vector<int> candidate_faces_ortho_horizontal;
  std::vector<int> candidate_faces_ortho_vertical;

  for (int i = 0; i < faces_model.size(); i++) {
    // Ignore duplicated planes or very small areas
    if (std::find(duplicated_faces.begin(), duplicated_faces.end(), i) !=
            duplicated_faces.end() ||
        area.at(i) < 0.01) {
      continue;
    }

    Eigen::Vector3d candidate_normal = plane_normals.at(i);
    double candidate_d = plane_d.at(i);

    if (std::abs(element_normal.dot(candidate_normal)) < 0.2) {
      std::vector<uint32_t> vertices = faces_model[i].vertices;
      double min_distance = 1000;
      for (int p_idx = 0; p_idx < element_points->size(); p_idx++) {
        pcl::PointXYZ p = (*element_points)[p_idx];
        double error =
            std::abs(candidate_normal.x() * p.x + candidate_normal.y() * p.y +
                     candidate_normal.z() * p.z + candidate_d);
        if (error < min_distance) {
          min_distance = error;
        }
      }
      if (min_distance < 1.0) {
        if (std::abs(candidate_normal.z()) > 0.5) {
          candidate_faces_ortho_vertical.push_back(i);
        } else {
          candidate_faces_ortho_horizontal.push_back(i);
        }
      }
    }
  }

  int nr_main = candidate_faces_main.size();
  int nr_ortho_vertical = candidate_faces_ortho_vertical.size();
  int nr_ortho_horizontal = candidate_faces_ortho_horizontal.size();
  std::cout << "Number of main: " << nr_main << std::endl;
  std::cout << "Number of ortho_vertical: " << nr_ortho_vertical << std::endl;
  std::cout << "Number of ortho_horizontal: " << nr_ortho_horizontal
            << std::endl;

  // Compute combinations
  std::vector<std::vector<int>> candidates_main;
  std::vector<std::vector<int>> candidates_ortho_vertical;
  std::vector<std::vector<int>> candidates_ortho_horizontal;

  for (int i = 0; i < candidate_faces_main.size(); i++) {
    for (int j = i + 1; j < candidate_faces_main.size(); j++) {
      std::vector<int> candidate_temp;
      candidate_temp.push_back(candidate_faces_main.at(i));
      candidate_temp.push_back(candidate_faces_main.at(j));
      candidates_main.push_back(candidate_temp);
    }
  }

  for (int i = 0; i < candidate_faces_ortho_vertical.size(); i++) {
    for (int j = i + 1; j < candidate_faces_ortho_vertical.size(); j++) {
      std::vector<int> candidate_temp;
      candidate_temp.push_back(candidate_faces_ortho_vertical.at(i));
      candidate_temp.push_back(candidate_faces_ortho_vertical.at(j));
      candidates_ortho_vertical.push_back(candidate_temp);
    }
  }
  for (int i = 0; i < candidate_faces_ortho_horizontal.size(); i++) {
    for (int j = i + 1; j < candidate_faces_ortho_horizontal.size(); j++) {
      std::vector<int> candidate_temp;
      candidate_temp.push_back(candidate_faces_ortho_horizontal.at(i));
      candidate_temp.push_back(candidate_faces_ortho_horizontal.at(j));
      candidates_ortho_horizontal.push_back(candidate_temp);
    }
  }

  // Fuse differnt combinations together
  std::vector<std::vector<int>> candidates;
  for (int l1 = 0; l1 < candidates_main.size(); l1++) {
    for (int l2 = 0; l2 < candidates_ortho_vertical.size(); l2++) {
      for (int l3 = 0; l3 < candidates_ortho_horizontal.size(); l3++) {
        std::vector<int> candidate;
        candidate.push_back(candidates_main.at(l1).at(0));
        candidate.push_back(candidates_main.at(l1).at(1));

        candidate.push_back(candidates_ortho_vertical.at(l2).at(0));
        candidate.push_back(candidates_ortho_vertical.at(l2).at(1));

        candidate.push_back(candidates_ortho_horizontal.at(l3).at(0));
        candidate.push_back(candidates_ortho_horizontal.at(l3).at(1));

        candidates.push_back(candidate);
      }
    }
  }

  int nr_candidates = candidates.size();
  std::cout << "Number of candidates: " << nr_candidates << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr reconstructed_element(
      new pcl::PointCloud<pcl::PointXYZ>);

  for (int i = 0; i < nr_candidates; i++) {
    std::vector<int> candidats_idx = candidates.at(i);

    Eigen::Vector4f plane_m1, plane_m2, plane_v1, plane_v2, plane_h1, plane_h2;

    planeFromIdx(plane_m1, candidats_idx.at(0), plane_normals, plane_d);
    planeFromIdx(plane_m2, candidats_idx.at(1), plane_normals, plane_d);
    planeFromIdx(plane_v1, candidats_idx.at(2), plane_normals, plane_d);
    planeFromIdx(plane_v2, candidats_idx.at(3), plane_normals, plane_d);
    planeFromIdx(plane_h1, candidats_idx.at(4), plane_normals, plane_d);
    planeFromIdx(plane_h2, candidats_idx.at(5), plane_normals, plane_d);

    // All intersections with m1
    Eigen::Vector3f p1, p2, p3, p4, p5, p6, p7, p8;
    pcl::threePlanesIntersection(plane_m1, plane_v1, plane_h1, p1);
    pcl::threePlanesIntersection(plane_m1, plane_v1, plane_h2, p2);
    pcl::threePlanesIntersection(plane_m1, plane_v2, plane_h1, p3);
    pcl::threePlanesIntersection(plane_m1, plane_v2, plane_h2, p4);

    // All intersections with m2
    pcl::threePlanesIntersection(plane_m2, plane_v1, plane_h1, p5);
    pcl::threePlanesIntersection(plane_m2, plane_v1, plane_h2, p6);
    pcl::threePlanesIntersection(plane_m2, plane_v2, plane_h1, p7);
    pcl::threePlanesIntersection(plane_m2, plane_v2, plane_h2, p8);

    reconstructed_element->push_back(pcl::PointXYZ(p1.x(), p1.y(), p1.z()));
    reconstructed_element->push_back(pcl::PointXYZ(p2.x(), p2.y(), p2.z()));
    reconstructed_element->push_back(pcl::PointXYZ(p3.x(), p3.y(), p3.z()));
    reconstructed_element->push_back(pcl::PointXYZ(p4.x(), p4.y(), p4.z()));
    reconstructed_element->push_back(pcl::PointXYZ(p5.x(), p5.y(), p5.z()));
    reconstructed_element->push_back(pcl::PointXYZ(p6.x(), p6.y(), p6.z()));
    reconstructed_element->push_back(pcl::PointXYZ(p7.x(), p7.y(), p7.z()));
    reconstructed_element->push_back(pcl::PointXYZ(p8.x(), p8.y(), p8.z()));
  }
  pcl::io::savePLYFile("/home/philipp/Schreibtisch/reconstructed.ply",
                       *reconstructed_element);
  /*
  MIP_Solver solver;
  std::vector<Variable*> variables = solver.create_variables(nr_candidates);

  //Set constraints
  Linear_constraint* c1 = solver.create_constraint(2, 2, "c1");
  for (int i = 0; i < nr_main; i++){
    variables.at(i)->set_variable_type(Variable::BINARY);
    c1->add_coefficient(variables.at(i), 1);
  }

  //Set objective
  Linear_objective * obj = solver.create_objective(Linear_objective::MAXIMIZE);
  for (int i = 0; i < candidate_faces_main.size(); i++){
    double l1 = 0;

    //Coverage
    int idx = candidate_faces_main.at(i);

    Eigen::Vector3d plane_normal_1 = plane_normals.at(idx);
    double a_1 = plane_normal_1.x();
    double b_1 = plane_normal_1.y();
    double c_1 = plane_normal_1.z();
    double d_1 = plane_d.at(idx);

    for (int n = 0; n < element_points->size(); n++){
      pcl::PointXYZ p = (*element_points)[n];
      double error = a_1 * p.x + b_1 * p.y + c_1 * p.z + d_1;
      l1 -= (error * error);
    }
    obj->add_coefficient(variables_main.at(i), l1);
  }
  for (int i = 0; i < candidate_faces_ortho.size(); i++){
    double l1 = 0;

    //Coverage
    int idx = candidate_faces_ortho.at(i);

    Eigen::Vector3d plane_normal_1 = plane_normals.at(idx);
    double a_1 = plane_normal_1.x();
    double b_1 = plane_normal_1.y();
    double c_1 = plane_normal_1.z();
    double d_1 = plane_d.at(idx);

    for (int n = 0; n < element_points->size(); n++){
      pcl::PointXYZ p = (*element_points)[n];
      double error = a_1 * p.x + b_1 * p.y + c_1 * p.z + d_1;
      l1 -= (error * error);
    }
    obj->add_coefficient(variables_ortho.at(i), l1);
  }


  solver.solve();
  const std::vector<double>& results = solver.solution();

  candidate_faces_main.insert(candidate_faces_main.end(),
  candidate_faces_ortho.begin(), candidate_faces_ortho.end());
  pcl::PointCloud<pcl::PointXYZ>::Ptr reconstructed_element (new
  pcl::PointCloud<pcl::PointXYZ>); for (std::size_t i = 0; i < results.size();
  ++i) { if (results[i] != 0){ std::cout << "\tx" << i + 1 << ": " << results[i]
  << std::endl;

      int idx = candidate_faces_main.at(i);
      std::vector<uint32_t> vertices = faces_model[idx].vertices;

      for (int j = 0; j < 3; j++){
        reconstructed_element->push_back((*vertices_model)[vertices.at(j)]);
      }
    }
  }
  pcl::io::savePLYFile("/home/philipp/Schreibtisch/reconstructed.ply",*reconstructed_element);
  */

  return 0;
}

/*
std::vector<::pcl::Vertices> faces = mesh.polygons;
pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>);
pcl::fromPCLPointCloud2(mesh.cloud, *points);

std::cout << "Number faces: " << faces.size() << std::endl;

std::vector<int> blocking_idx;
std::vector<std::pair<int, int>> quads;

//Group elements sharing the a half edge and have the same surface normal
for (int i = 0; i < faces.size(); i++){
if (std::find(blocking_idx.begin(), blocking_idx.end(), i) !=
blocking_idx.end()) {
continue;
}
std::vector<uint32_t> vertices_1 = faces[i].vertices;

pcl::PointXYZ p1_1 = (*points)[vertices_1.at(0)];
pcl::PointXYZ p1_2 = (*points)[vertices_1.at(1)];
pcl::PointXYZ p1_3 = (*points)[vertices_1.at(2)];
Eigen::Vector3d v1_1(p1_1.x, p1_1.y, p1_1.z);
Eigen::Vector3d v1_2(p1_2.x, p1_2.y, p1_2.z);
Eigen::Vector3d v1_3(p1_3.x, p1_3.y, p1_3.z);
double l1_1 = (v1_1 - v1_2).norm();
double l1_2 = (v1_2 - v1_3).norm();
double l1_3 = (v1_3 - v1_1).norm();
double l1_max = (l1_1 > l1_2) ? l1_1 : l1_2;
l1_max = (l1_3 > l1_max) ? l1_3 : l1_max;

double area = ((v1_1 - v1_2).cross(v1_3 - v1_2)).norm();

for (int j = i + 1; j < faces.size(); j++){
if (std::find(blocking_idx.begin(), blocking_idx.end(), j) !=
blocking_idx.end() || area < 0.05) {
continue;
}
std::vector<uint32_t> vertices_2 = faces[j].vertices;
int v_matches = 0;

// Check if two points matches exactly
//TODO: Assumption that vertices do not exist twice!
for (int n = 0; n < 3; n++){
for (int m = 0; m < 3; m++){
if (vertices_1.at(n) == vertices_2.at(m)){
v_matches++;
break;
}
}
}

if(v_matches == 2){
pcl::PointXYZ p2_1 = (*points)[vertices_2.at(0)];
pcl::PointXYZ p2_2 = (*points)[vertices_2.at(1)];
pcl::PointXYZ p2_3 = (*points)[vertices_2.at(2)];
Eigen::Vector3d v2_1(p2_1.x, p2_1.y, p2_1.z);
Eigen::Vector3d v2_2(p2_2.x, p2_2.y, p2_2.z);
Eigen::Vector3d v2_3(p2_3.x, p2_3.y, p2_3.z);
double l2_1 = (v2_1 - v2_2).norm();
double l2_2 = (v2_2 - v2_3).norm();
double l2_3 = (v2_3 - v2_1).norm();
double l2_max = (l2_1 > l2_2) ? l2_1 : l2_2;
l2_max = (l2_3 > l2_max) ? l2_3 : l2_max;

//Matching half edge
if(std::abs(l1_max - l2_max) < 10e-15){
// Check if normal is the same
Eigen::Vector3d n1 = (v1_1- v1_2).cross(v1_3 - v1_2);
Eigen::Vector3d n2 = (v2_1- v2_2).cross(v2_3 - v2_2);
n1.normalize();
n2.normalize();

//Check for same normal
if ( (n1 - n2).lpNorm<Eigen::Infinity>() < 10e-15 || (n1 +
n2).lpNorm<Eigen::Infinity>() < 10e-15){
//Match found
blocking_idx.push_back(i);
blocking_idx.push_back(j);
quads.push_back(std::make_pair(i, j));
break;
}
}
}
}
}

std::cout << "Size of pairs: " << quads.size() << std::endl;
pcl::PolygonMesh mesh_all;
for (int i = 0; i < quads.size(); i++){
std::pair<int,int> p = quads.at(i);

pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
std::vector<uint32_t> vertices_1 = faces[p.first].vertices;
std::vector<uint32_t> vertices_2 = faces[p.second].vertices;

//Start at a correct edge!!
pcl::PointXYZ p1 = (*points)[vertices_1.at(0)];
pcl::PointXYZ p2 = (*points)[vertices_1.at(1)];
pcl::PointXYZ p3 = (*points)[vertices_1.at(2)];
Eigen::Vector3d v1(p1.x, p1.y, p1.z);
Eigen::Vector3d v2(p2.x, p2.y, p2.z);
Eigen::Vector3d v3(p3.x, p3.y, p3.z);
double l1 = (v1 - v2).norm();
double l2 = (v2 - v3).norm();
double l3 = (v3 - v1).norm();

int start_point;
if (l1 > l2 && l1 > l3){
start_point = 1;
} else if (l2 > l1 && l2 > l3){
start_point = 2;
} else if (l3 > l1 && l3 > l1){
start_point = 3;
} else{
std::cout << "Error in starting point" << std::endl;
}


for (int j = 0; j < 3; j++){
int idx = vertices_1.at(j);
plane->push_back((*points)[idx]);
}

for (int j = 0; j < 3; j++){
int idx = vertices_2.at(j);
if (std::find(vertices_1.begin(), vertices_1.end(), idx) == vertices_1.end()) {
plane->push_back((*points)[idx]);
}
}

pcl::PolygonMesh mesh;
pcl::PCLPointCloud2 temp_cloud;
pcl::toPCLPointCloud2(*plane, temp_cloud);
std::vector<pcl::Vertices> polygons_new;
pcl::Vertices vertices_new;

if (start_point == 1){
vertices_new.vertices.push_back(0);
vertices_new.vertices.push_back(3);
vertices_new.vertices.push_back(1);
vertices_new.vertices.push_back(2);
} else if (start_point == 2){
vertices_new.vertices.push_back(0);
vertices_new.vertices.push_back(1);
vertices_new.vertices.push_back(3);
vertices_new.vertices.push_back(2);
} else if (start_point == 3){
vertices_new.vertices.push_back(0);
vertices_new.vertices.push_back(1);
vertices_new.vertices.push_back(2);
vertices_new.vertices.push_back(3);
}
polygons_new.push_back(vertices_new);
mesh.polygons = polygons_new;
mesh.cloud = temp_cloud;

combineMeshes(mesh, mesh_all);
}
pcl::io::savePLYFile("/home/philipp/Schreibtisch/mesh_all2.ply",mesh_all);
 */