#ifndef CPT_RECONSTRUCTION_MODEL_H
#define CPT_RECONSTRUCTION_MODEL_H

#include "ros/ros.h"

#include <vector>

#include <pcl/features/normal_3d.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/IO/write_xyz_points.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/property_map.h>
#include <CGAL/tags.h>

#include <CGAL/IO/Writer_OFF.h>
#include <CGAL/Polygonal_surface_reconstruction.h>
#include <CGAL/Surface_mesh.h>
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
typedef CGAL::Shape_detection::Cone<Traits> Cone;
typedef CGAL::Shape_detection::Cylinder<Traits> Cylinder;
typedef CGAL::Shape_detection::Plane<Traits> Plane;
typedef CGAL::Shape_detection::Sphere<Traits> Sphere;
typedef CGAL::Shape_detection::Torus<Traits> Torus;
// typedef CGAL::Parallel_tag Concurrency_tag;

namespace cad_percept {
namespace cpt_reconstruction {
class Model {
 public:
  Model() = delete;
  Model(std::string filename, Eigen::Matrix4d transformation);

  void preprocess();
  void queryTree(pcl::PointXYZ p);
  void addOutlier(pcl::PointXYZ p);
  void addNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                  pcl::PointCloud<pcl::Normal>::Ptr normals, int k);
  void efficientRANSAC();
  void SACSegmentation();
  void applyFilter();

  std::vector<Eigen::MatrixXd>* getPointShapes();
  std::vector<Eigen::MatrixXd>* getNormalShapes();
  std::vector<Eigen::Vector3d>* getRansacNormals();
  std::vector<int>* getShapeIDs();

  int getOutlierCount();
  float getMinDistance();
  int getIndex();
  void clearRansacShapes();
  void clearBuffer();

  void printOutliers();

 private:
  Eigen::Matrix4d transformation_;
  std::string filename_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr model_points_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr meshing_points_;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTree_;
  std::vector<int> nn_indices_{1};
  std::vector<float> nn_dists_{1};
  std::vector<int> idx_outliers_;  // TODO REMOVE
  std::vector<Eigen::MatrixXd> points_shape_;
  std::vector<Eigen::MatrixXd> normals_shape_;
  std::vector<Eigen::Vector3d> ransac_normals_;
  std::vector<int> shape_id_;
  std::vector<Eigen::Vector4d> param_plane_;
  std::vector<Eigen::Vector4d> param_cyl_;
};
}  // namespace cpt_reconstruction
}  // namespace cad_percept
#endif  // CPT_RECONSTRUCTION_MODEL_H
