#include <cpt_reconstruction/reconstruction_mesh_generation.h>

#include <geometry_msgs/Vector3.h>
#include "cpt_reconstruction/shape.h"
#include "std_msgs/String.h"

#include <pcl/surface/concave_hull.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/ply_io.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/poisson_surface_reconstruction.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/IO/read_xyz_points.h>
#include <vector>
#include <fstream>

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

//Source: https://pointclouds.org/documentation/tutorials/greedy_projection.html
// https://pointclouds.org/documentation/tutorials/resampling.html
void MeshGeneration::messageCallback(const ::cpt_reconstruction::shape& msg) {
  ROS_INFO("[Mesh Generation] Id: %d with size: %d", msg.id,
           msg.points_msg.size());

  pcl::PointCloud<pcl::PointXYZ>::Ptr points_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  //std::vector<Point_with_normal> points_with_normals(msg.points_msg.size());
  std::vector<geometry_msgs::Vector3> points = msg.points_msg;
  //std::vector<geometry_msgs::Vector3> normals = msg.normals_msg;
  geometry_msgs::Vector3 ransac_normal_temp = msg.ransac_normal;
  Eigen::Vector3d ransac_normal(ransac_normal_temp.x, ransac_normal_temp.y, ransac_normal_temp.z);
  for (unsigned i = 0; i < points.size(); i++) {
    geometry_msgs::Vector3 p = points.at(i);
    //geometry_msgs::Vector3 n = normals.at(i);

    Kernel::Point_3 p_temp(p.x, p.y, p.z);
    //Kernel::Vector_3 n_temp(n.x, n.y, n.z);
    //std::pair<Kernel::Point_3, Kernel::Vector_3> res(p_temp, n_temp);
    //points_with_normals[i] = res;

    points_cloud->push_back(pcl::PointXYZ(p.x, p.y, p.z));
    //normals_cloud->push_back(pcl::Normal(n.x, n.y, n.z));
  }



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

    if(diff1 < 0.15 || diff2 < 0.15) {
      for (unsigned j = 0; j < points_cloud->size(); j++) {
        pcl::PointXYZ p = (*points_cloud)[j];
        kd_trees_.at(i)->nearestKSearch(p, 1, nn_indices, nn_dists);
        if (nn_dists[0] < 0.02) {
          matches++;
        }
      }
      double coverage = ((double) matches) / ((double) point_cloud_size);
      if (coverage > 0.3) {
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

      pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZ>());
      kd_tree->setInputCloud(clouds_[idx]);
      kd_trees_.at(idx) = kd_tree;
    } else{
      ROS_INFO("Fuse multiple elements\n");
      for (unsigned j = 0; j < matching_idx.size(); j++){
        int idx = matching_idx.at(j);
        *points_cloud += (*(clouds_[idx]));
      }
      clouds_.push_back(points_cloud);
      pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZ>());
      kd_tree->setInputCloud(points_cloud);
      kd_trees_.push_back(kd_tree);
      ransac_normals_.push_back(ransac_normal);

      int idx_corr = 0;
      for (unsigned r = 0; r < matching_idx.size(); r++){
        clouds_.erase(clouds_.begin() + matching_idx.at(r) - idx_corr);
        kd_trees_.erase(kd_trees_.begin() + matching_idx.at(r) - idx_corr);
        idx_corr++;
      }
    }
  }
  else {
    clouds_.push_back(points_cloud);
    ransac_normals_.push_back(ransac_normal);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZ>());
    kd_tree->setInputCloud(points_cloud);
    kd_trees_.push_back(kd_tree);
  }

  ROS_INFO("[Mesh Generation] Counter: %d \n", counter_);
  if (counter_ >= 200 && (counter_ % 200 == 0)){

    // TODO: Check if concave hull volume changed

    //this->purgeBuffer();
    for (unsigned i = 0; i < clouds_.size(); i++){

      //if (clouds_[i]->size() < 300){
      //  continue;
      //}
      /*
      pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>);
      pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
      mls.setComputeNormals (true);
      mls.setInputCloud (clouds_[i]);
      mls.setPolynomialOrder (2);
      mls.setSearchMethod (tree);
      mls.setSearchRadius (0.8);
      mls.process (*mls_points);
      */
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::ConcaveHull<pcl::PointXYZ> chull;
      chull.setInputCloud (clouds_[i]);
      chull.setAlpha (0.1);
      chull.reconstruct (*cloud_hull);

      std::string result = "/home/philipp/Schreibtisch/Meshes/mesh_" +
                           std::to_string(i) + ".ply";
      pcl::io::savePLYFile(result, *cloud_hull);
    }
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

void MeshGeneration::purgeBuffer(){
  //TODO !!
  for (int i = 0; i < clouds_.size(); i++){
    // maybe reconstruct it with convex hull
    if (clouds_[i]->size() < 500){
      clouds_.erase(clouds_.begin() + i);
      kd_trees_.erase(kd_trees_.begin() + i);
      ransac_normals_.erase(ransac_normals_.begin() + i);
    }
  }
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
pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>);
pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
mls.setComputeNormals (true);
mls.setInputCloud (points_cloud);
mls.setPolynomialOrder (2);
mls.setSearchMethod (tree);
mls.setSearchRadius (0.5);
mls.process (*mls_points);

pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
tree2->setInputCloud (mls_points);

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