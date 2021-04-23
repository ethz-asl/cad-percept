#include <cpt_reconstruction/reconstruction_mesh_generation.h>

#include <geometry_msgs/Vector3.h>
#include <algorithm>
#include <fstream>
#include <vector>

#include "cpt_reconstruction/shape.h"
#include "std_msgs/String.h"

#include <pcl/PCLHeader.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PolygonMesh.h>
#include <pcl/Vertices.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
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
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/IO/read_xyz_points.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/poisson_surface_reconstruction.h>

#include <Eigen/Dense>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef std::pair<Kernel::Point_3, Kernel::Vector_3> Point_with_normal;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;

namespace cad_percept {
namespace cpt_reconstruction {
MeshGeneration::MeshGeneration(ros::NodeHandle nodeHandle)
    : nodeHandle_(nodeHandle), counter_(0) {
  subscriber_ = nodeHandle_.subscribe("ransac_shape", 1000,
                                      &MeshGeneration::messageCallback, this);
  ros::spin();
}

void MeshGeneration::combineMeshes(const pcl::PolygonMesh &mesh,
                                   pcl::PolygonMesh &mesh_all) {
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

void MeshGeneration::fit3DBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                              pcl::PolygonMesh &mesh) {
  // Compute LS plane
  /*
  int num_points = cloud->size();
  Eigen::MatrixXd A(num_points, 4);
  Eigen::VectorXd B(num_points);
  for (int i = 0; i < num_points; i++){
    pcl::PointXYZ p = (*cloud)[i];
    A.block<1, 4>(i, 0) = Eigen::Vector4d(p.x, p.y, p.z, 1.0);
    B(i) = 0;
  }

  Eigen::Vector4d plane_coeff = A.bdcSvd(Eigen::ComputeThinU |
  Eigen::ComputeThinV).solve(B); Eigen::Vector3d normal(plane_coeff(0),
  plane_coeff(1), plane_coeff(2)); normal.normalize();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new
  pcl::PointCloud<pcl::PointXYZ>); for (int i = 0; i < num_points; i++){
    pcl::PointXYZ p_pcl = (*cloud)[i];
    Eigen::Vector3d p(p_pcl.x, p_pcl.y, p_pcl.z);
    Eigen::Vector3d p_proj = p - (p.dot(normal) + plane_coeff(3)) * normal;
    pcl::PointXYZ p_pcl_proj(p_proj.x(), p_proj.y(), p_proj.z());
    cloud_projected->push_back(p_pcl_proj);
  }
  cloud = cloud_projected;
   */

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_copy(
      new pcl::PointCloud<pcl::PointXYZ>);
  *cloud_copy += *cloud;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_new(
      new pcl::PointCloud<pcl::PointXYZ>);

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);
  seg.setMaxIterations(1000);

  int i = 0, nr_points = (int)cloud_copy->points.size();
  while (cloud_copy->points.size() > 0.3 * nr_points) {
    seg.setInputCloud(cloud_copy);
    seg.segment(*inliers, *coefficients);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(
        new pcl::PointCloud<pcl::PointXYZ>);
    extract.setInputCloud(cloud_copy);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_temp);
    *cloud_new += *cloud_temp;

    extract.setInputCloud(cloud_copy);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_copy);
  }

  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  searchTree->setInputCloud(cloud_new);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
  normalEstimator.setInputCloud(cloud_new);
  normalEstimator.setSearchMethod(searchTree);
  normalEstimator.setKSearch(10);
  normalEstimator.compute(*normals);

  pcl::PointCloud<pcl::PointXYZI>::Ptr corners(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal> harris;
  harris.setInputCloud(cloud_new);
  harris.setNonMaxSupression(false);
  harris.setRadius(0.2f);
  harris.setThreshold(0.1f);
  harris.setNormals(normals);
  harris.compute(*corners);
  // Project Points to plane

  // Get linear boundary

  // Extrude element

  // Compute corner_points

  // Compute Convex? or Concave Hull
  pcl::ConvexHull<pcl::PointXYZI> chull;
  chull.setInputCloud(corners);
  // chull.setAlpha (0.1);
  chull.reconstruct(mesh);

  // Source:
  // https://stackoverflow.com/questions/49688940/point-cloud-library-rotation-of-axis-alligned-bounding-box
  // and https://pcl.readthedocs.io/en/latest/moment_of_inertia.html
  // Bounding box
  //
  /*
  pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud(clouds_[i]);
  feature_extractor.compute();
  pcl::PointXYZ min_point_OBB;
  pcl::PointXYZ max_point_OBB;
  pcl::PointXYZ position_OBB;
  Eigen::Matrix3f rotational_matrix_OBB;
  feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB,
                           rotational_matrix_OBB);
  Eigen::Vector3f p1(min_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
  Eigen::Vector3f p2(min_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
  Eigen::Vector3f p3(max_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
  Eigen::Vector3f p4(max_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
  Eigen::Vector3f p5(min_point_OBB.x, max_point_OBB.y, min_point_OBB.z);
  Eigen::Vector3f p6(min_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
  Eigen::Vector3f p7(max_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
  Eigen::Vector3f p8(max_point_OBB.x, max_point_OBB.y, min_point_OBB.z);
  Eigen::Vector3f position(position_OBB.x, position_OBB.y,
                           position_OBB.z);
  p1 = rotational_matrix_OBB * p1 + position;
  p2 = rotational_matrix_OBB * p2 + position;
  p3 = rotational_matrix_OBB * p3 + position;
  p4 = rotational_matrix_OBB * p4 + position;
  p5 = rotational_matrix_OBB * p5 + position;
  p6 = rotational_matrix_OBB * p6 + position;
  p7 = rotational_matrix_OBB * p7 + position;
  p8 = rotational_matrix_OBB * p8 + position;
  pcl::PointCloud<pcl::PointXYZ>::Ptr box_cloud(
          new pcl::PointCloud<pcl::PointXYZ>);
  box_cloud->push_back(pcl::PointXYZ(p1.x(), p1.y(), p1.z()));
  box_cloud->push_back(pcl::PointXYZ(p2.x(), p2.y(), p2.z()));
  box_cloud->push_back(pcl::PointXYZ(p3.x(), p3.y(), p3.z()));
  box_cloud->push_back(pcl::PointXYZ(p4.x(), p4.y(), p4.z()));
  box_cloud->push_back(pcl::PointXYZ(p5.x(), p5.y(), p5.z()));
  box_cloud->push_back(pcl::PointXYZ(p6.x(), p6.y(), p6.z()));
  box_cloud->push_back(pcl::PointXYZ(p7.x(), p7.y(), p7.z()));
  box_cloud->push_back(pcl::PointXYZ(p8.x(), p8.y(), p8.z()));
  */
}

// Source:
// https://pointclouds.org/documentation/tutorials/greedy_projection.html
// https://pointclouds.org/documentation/tutorials/resampling.html
void MeshGeneration::messageCallback(const ::cpt_reconstruction::shape &msg) {
  ROS_INFO("[Mesh Generation] Id: %d with size: %d", msg.id,
           msg.points_msg.size());

  pcl::PointCloud<pcl::PointXYZ>::Ptr points_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  // std::vector<Point_with_normal> points_with_normals(msg.points_msg.size());
  std::vector<geometry_msgs::Vector3> points = msg.points_msg;
  // std::vector<geometry_msgs::Vector3> normals = msg.normals_msg;
  geometry_msgs::Vector3 ransac_normal_temp = msg.ransac_normal;
  Eigen::Vector3d ransac_normal(ransac_normal_temp.x, ransac_normal_temp.y,
                                ransac_normal_temp.z);
  for (unsigned i = 0; i < points.size(); i++) {
    geometry_msgs::Vector3 p = points.at(i);
    // geometry_msgs::Vector3 n = normals.at(i);

    Kernel::Point_3 p_temp(p.x, p.y, p.z);
    // Kernel::Vector_3 n_temp(n.x, n.y, n.z);
    // std::pair<Kernel::Point_3, Kernel::Vector_3> res(p_temp, n_temp);
    // points_with_normals[i] = res;

    points_cloud->push_back(pcl::PointXYZ(p.x, p.y, p.z));
    // normals_cloud->push_back(pcl::Normal(n.x, n.y, n.z));
  }

  if (points_cloud->size() > 100) {
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(points_cloud);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*points_cloud);

    clouds_.push_back(points_cloud);
    ransac_normals_.push_back(ransac_normal);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree(
        new pcl::search::KdTree<pcl::PointXYZ>());
    kd_tree->setInputCloud(points_cloud);
    kd_trees_.push_back(kd_tree);
    fusing_count_.push_back(1);

    if (counter_ >= 50 && (counter_ % 50 == 0)) {
      ROS_INFO("Size before fuseing: %d \n", clouds_.size());
      this->fusePlanes();
      ROS_INFO("Size after fuseing: %d \n", clouds_.size());
      // this->fusePlanes();
      // ROS_INFO("Size after fuseing: %d \n", clouds_.size());
    }
    if (counter_ >= 200 && (counter_ % 200 == 0)) {
      ROS_INFO("Size before removing single detections: %d \n", clouds_.size());
      this->removeSingleDetections();
      ROS_INFO("Size after removing single detections: %d \n", clouds_.size());
    }

    ROS_INFO("[Mesh Generation] Counter: %d \n", counter_);
    if (counter_ >= 400 && (counter_ % 400 == 0)) {
      pcl::PolygonMesh mesh_all;
      for (unsigned i = 0; i < clouds_.size(); i++) {
        if (fusing_count_.at(i) > 3) {
          pcl::PolygonMesh mesh;
          this->fit3DBox(clouds_[i], mesh);
          this->combineMeshes(mesh, mesh_all);

          std::string result = "/home/philipp/Schreibtisch/Meshes/points_" +
                               std::to_string(i) + ".ply";
          pcl::io::savePLYFile(result, *clouds_[i]);

          std::string result2 = "/home/philipp/Schreibtisch/Meshes/mesh_" +
                                std::to_string(i) + ".ply";
          pcl::io::savePLYFile(result2, mesh);
        }
        /*
        pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new
        pcl::PointCloud<pcl::PointNormal>);
        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new
        pcl::search::KdTree<pcl::PointXYZ>); mls.setComputeNormals (true);
        mls.setInputCloud (clouds_[i]);
        mls.setPolynomialOrder (2);
        mls.setSearchMethod (tree);
        mls.setSearchRadius (0.8);
        mls.process (*mls_points);
        */
      }
      pcl::io::savePLYFile("/home/philipp/Schreibtisch/Meshes/mesh_all.ply",
                           mesh_all);
    }
    /*
    if (msg.id == 0){
    } else if (msg.id == 0){
    } else {
      ROS_INFO("Unknown shape\n");
    }
    */

    counter_++;
  }
}

void MeshGeneration::removeSingleDetections() {
  std::vector<int> remove_idx;
  for (int i = 0; i < fusing_count_.size(); i++) {
    if (fusing_count_.at(i) <= 1 || clouds_.at(i)->size() < 500) {
      remove_idx.push_back(i);
    }
  }
  int idx_corr = 0;
  for (unsigned r = 0; r < remove_idx.size(); r++) {
    int idx = remove_idx.at(r);
    clouds_.erase(clouds_.begin() + idx - idx_corr);
    kd_trees_.erase(kd_trees_.begin() + idx - idx_corr);
    ransac_normals_.erase(ransac_normals_.begin() + idx - idx_corr);
    fusing_count_.erase(fusing_count_.begin() + idx - idx_corr);
    idx_corr++;
  }
}

void MeshGeneration::fusePlanes() {
  std::vector<int> blocked_idx;
  std::vector<int> nn_indices{1};
  std::vector<float> nn_dists{1};
  std::vector<std::vector<int>> fusing;
  for (int i = 0; i < clouds_.size(); i++) {
    if (std::find(blocked_idx.begin(), blocked_idx.end(), i) !=
        blocked_idx.end()) {
      continue;
    }
    std::vector<int> fusing_temp;
    fusing_temp.push_back(i);
    for (int j = i + 1; j < clouds_.size(); j++) {
      if (std::find(blocked_idx.begin(), blocked_idx.end(), j) !=
          blocked_idx.end()) {
        continue;
      }
      double diff1 =
          (ransac_normals_[i] - ransac_normals_[j]).lpNorm<Eigen::Infinity>();
      double diff2 =
          (ransac_normals_[i] + ransac_normals_[j]).lpNorm<Eigen::Infinity>();
      if (diff1 < 0.05 || diff2 < 0.05) {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_i = kd_trees_[i];
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_j = clouds_[j];
        int matches = 0;
        for (int k = 0; k < cloud_j->size(); k += 2) {
          pcl::PointXYZ p = (*cloud_j)[k];
          tree_i->nearestKSearch(p, 1, nn_indices, nn_dists);
          if (nn_dists[0] < 0.01) {
            matches++;
          }
        }
        double coverage = 2.0 * ((double)matches) / ((double)cloud_j->size());
        if (coverage >= 0.3) {
          blocked_idx.push_back(j);
          fusing_temp.push_back(j);
        }
      }
    }
    fusing.push_back(fusing_temp);
  }
  std::vector<pcl::search::KdTree<pcl::PointXYZ>::Ptr> fused_kd_trees;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> fused_clouds;
  std::vector<Eigen::Vector3d> fused_ransac_normals;
  std::vector<int> fusing_count;

  for (int i = 0; i < fusing.size(); i++) {
    std::vector<int> fusing_vec = fusing.at(i);
    pcl::PointCloud<pcl::PointXYZ>::Ptr fused_point_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    int count_fused_shapes = 0;
    for (int j = 0; j < fusing_vec.size(); j++) {
      (*fused_point_cloud) += (*(clouds_[fusing_vec.at(j)]));
      count_fused_shapes += fusing_count_.at(fusing_vec.at(j));
    }
    if (fused_point_cloud->size() > 100) {
      pcl::VoxelGrid<pcl::PointXYZ> sor;
      sor.setInputCloud(fused_point_cloud);
      sor.setLeafSize(0.01f, 0.01f, 0.01f);
      sor.filter(*fused_point_cloud);

      //
      // Recompute ransac normal and filter points
      // int size_b_SAC = fused_point_cloud->size();
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      pcl::ExtractIndices<pcl::PointXYZ> extract;

      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setDistanceThreshold(0.02);
      seg.setMaxIterations(1000);
      seg.setInputCloud(fused_point_cloud);
      seg.segment(*inliers, *coefficients);

      // extract.setInputCloud(fused_point_cloud);
      // extract.setIndices(inliers);
      // extract.setNegative(false);
      // extract.filter(*fused_point_cloud);

      Eigen::Vector3d plane_normal(coefficients->values[0],
                                   coefficients->values[1],
                                   coefficients->values[2]);
      plane_normal.normalize();
      //
      // int size_a_SAC = fused_point_cloud->size();
      // ROS_INFO("Size after SAC: %f \n", ((double)size_a_SAC) /
      // ((double)size_b_SAC));

      pcl::search::KdTree<pcl::PointXYZ>::Ptr fused_kd_tree(
          new pcl::search::KdTree<pcl::PointXYZ>());
      fused_kd_tree->setInputCloud(fused_point_cloud);
      fused_clouds.push_back(fused_point_cloud);
      fused_kd_trees.push_back(fused_kd_tree);
      // fused_ransac_normals.push_back(ransac_normals_[fusing_vec.at(0)]);
      fused_ransac_normals.push_back(plane_normal);
      fusing_count.push_back(count_fused_shapes);
    }
  }
  clouds_.clear();
  ransac_normals_.clear();
  kd_trees_.clear();
  fusing_count_.clear();
  clouds_ = fused_clouds;
  ransac_normals_ = fused_ransac_normals;
  kd_trees_ = fused_kd_trees;
  fusing_count_ = fusing_count;
}

}  // namespace cpt_reconstruction
}  // namespace cad_percept

//
/*
pcl::Poisson<pcl::PointNormal> poisson;
poisson.setDepth(10);
poisson.setInputCloud(mls_points);
pcl::PolygonMesh triangles;
poisson.reconstruct(triangles);
*/

/*
pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new
pcl::PointCloud<pcl::PointNormal>); pcl::MovingLeastSquares<pcl::PointXYZ,
pcl::PointNormal> mls; pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new
pcl::search::KdTree<pcl::PointXYZ>); mls.setComputeNormals (true);
mls.setInputCloud (points_cloud);
mls.setPolynomialOrder (2);
mls.setSearchMethod (tree);
mls.setSearchRadius (0.5);
mls.process (*mls_points);

pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new
pcl::search::KdTree<pcl::PointNormal>); tree2->setInputCloud (mls_points);

pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
pcl::PolygonMesh triangles;
gp3.setSearchRadius (0.5);
gp3.setMu (5);
gp3.setMaximumNearestNeighbors (100);
gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
gp3.setMinimumAngle(M_PI/18); // 10 degrees
gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
gp3.setNormalConsistency(false);

gp3.setInputCloud (mls_points);
gp3.setSearchMethod (tree2);
gp3.reconstruct (triangles);


std::string result = "/home/philipp/Schreibtisch/Meshes/mesh_" +
                     std::to_string(counter_) + ".ply";
pcl::io::savePLYFile (result, triangles);
*/

/*
bool existing_shape = false;
int point_cloud_size = points_cloud->size();
std::vector<int> matching_idx;
std::vector<int> nn_indices{1};
std::vector<float> nn_dists{1};
for (unsigned i = 0; i < kd_trees_.size(); i++){
  int matches = 0;

  Eigen::Vector3d ransac_normal_ref = ransac_normals_[i];
  double diff1 = (ransac_normal_ref - ransac_normal).lpNorm<Eigen::Infinity>();
  double diff2 = (ransac_normal_ref + ransac_normal).lpNorm<Eigen::Infinity>();

  if(diff1 < 0.1 || diff2 < 0.1) {
    for (unsigned j = 0; j < points_cloud->size(); j++) {
      pcl::PointXYZ p = (*points_cloud)[j];
      kd_trees_.at(i)->nearestKSearch(p, 1, nn_indices, nn_dists);
      if (nn_dists[0] < 0.05) {
        matches++;
      }
    }
    double coverage = ((double) matches) / ((double) point_cloud_size);
    if (coverage >= 0.15) {
      existing_shape = true;
      matching_idx.push_back(i);
    }
  }
}

if (existing_shape){
  if (matching_idx.size() == 1){
    ROS_INFO("Fuse one element\n");
    int idx = matching_idx.at(0);
    (*(clouds_[idx])) += *points_cloud;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree(new
pcl::search::KdTree<pcl::PointXYZ>()); kd_tree->setInputCloud(clouds_[idx]);
    kd_trees_.at(idx) = kd_tree;
  } else{
    ROS_INFO("Fuse multiple elements\n");
    for (unsigned j = 0; j < matching_idx.size(); j++){
      int idx = matching_idx.at(j);
      *points_cloud += (*(clouds_[idx]));
    }
    clouds_.push_back(points_cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree(new
pcl::search::KdTree<pcl::PointXYZ>()); kd_tree->setInputCloud(points_cloud);
    kd_trees_.push_back(kd_tree);
    ransac_normals_.push_back(ransac_normal);
    //TODO: Recompute ransac_normal?

    int idx_corr = 0;
    for (unsigned r = 0; r < matching_idx.size(); r++){
      clouds_.erase(clouds_.begin() + matching_idx.at(r) - idx_corr);
      kd_trees_.erase(kd_trees_.begin() + matching_idx.at(r) - idx_corr);
      ransac_normals_.erase(ransac_normals_.begin() + matching_idx.at(r) -
idx_corr); idx_corr++;
    }
  }
}
else {}
*/