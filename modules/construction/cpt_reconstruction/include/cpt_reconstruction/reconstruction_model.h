#ifndef CPT_RECONSTRUCTION_MODEL_H
#define CPT_RECONSTRUCTION_MODEL_H

#include "ros/ros.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <list>
#include <tuple>
#include <utility>
#include <vector>

#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/IO/Writer_OFF.h>
#include <CGAL/IO/write_xyz_points.h>
#include <CGAL/Polygonal_surface_reconstruction.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/property_map.h>
#include <CGAL/tags.h>
#include <cpt_reconstruction/reconstruction_custom_cylinder.h>
#include <cpt_reconstruction/reconstruction_custom_plane.h>
//#include <CGAL/GLPK_mixed_integer_program_traits.h>

// Type declarations.
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::FT FT;
typedef std::pair<Kernel::Point_3, Kernel::Vector_3> Point_with_normal;
typedef std::vector<Point_with_normal> Pwn_vector;
typedef CGAL::First_of_pair_property_map<Point_with_normal> Point_map;
typedef CGAL::Second_of_pair_property_map<Point_with_normal> Normal_map;
typedef CGAL::Shape_detection::Efficient_RANSAC_traits<Kernel, Pwn_vector,
                                                       Point_map, Normal_map>
    Traits;
typedef CGAL::Shape_detection::Efficient_RANSAC<Traits> Efficient_ransac;
typedef CGAL::Shape_detection::Custom_Cylinder<Traits> Cylinder;
typedef CGAL::Shape_detection::Custom_Plane<Traits> Plane;
// typedef CGAL::Parallel_tag Concurrency_tag;

namespace cad_percept {
namespace cpt_reconstruction {
class Model {
 public:
  Model() = delete;
  Model(const ros::NodeHandle &nodeHandle);

  void preprocess();
  void queryTree(pcl::PointXYZ p);
  void addOutlier(pcl::PointXYZ p);
  void addNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                  pcl::PointCloud<pcl::Normal>::Ptr normals, int k);

  void applyFilter();
  void efficientRANSAC();
  void SACSegmentation();

  std::vector<Eigen::MatrixXd>* getPointShapes();
  std::vector<Eigen::Vector3d>* getRansacNormals();
  std::vector<Eigen::Vector3d>* getAxis();
  std::vector<double>* getRadius();
  std::vector<int>* getShapeIDs();
  int getOutlierCount();
  double getMinDistance();
  void clearRansacShapes();
  void clearBuffer();

 private:
  std::string UPSAMPLED_BUILDING_MODEL_PATH_;
  bool USE_FILTER_;
  float OCTREE_FILTER_RESOLUTION_;
  double RANSAC_PROBABILITY_;
  int RANSAC_MIN_POINTS_;
  double RANSAC_EPSILON_;
  double RANSAC_CLUSTER_EPSILON_;
  double RANSAC_NORMAL_THRESHOLD_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr model_points_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr meshing_points_;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTree_;
  std::vector<int> nn_indices_{1};
  std::vector<float> nn_dists_{1};
  std::vector<Eigen::MatrixXd> points_shape_;
  std::vector<Eigen::Vector3d> ransac_normals_;
  std::vector<int> shape_id_;
  std::vector<Eigen::Vector3d> axis_;
  std::vector<double> radius_;
};
}  // namespace cpt_reconstruction
}  // namespace cad_percept
#endif  // CPT_RECONSTRUCTION_MODEL_H
