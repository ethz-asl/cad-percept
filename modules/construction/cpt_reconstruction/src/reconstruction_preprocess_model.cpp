#include <cpt_reconstruction/reconstruction_preprocess_model.h>

#include "ros/ros.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <list>
#include <tuple>
#include <utility>
#include <vector>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/IO/write_xyz_points.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/property_map.h>
#include <CGAL/tags.h>

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
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

namespace cad_percept {
namespace cpt_reconstruction {

PreprocessModel::PreprocessModel(std::string filename,
                                 Eigen::Matrix4d transformation)
    : meshing_points_(new pcl::PointCloud<pcl::PointXYZ>),
      filename_(filename),
      transformation_(transformation) {}

void PreprocessModel::preprocess() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr model_points(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PLYReader reader;
  reader.read(filename_, *model_points);
  model_points_ = model_points;

  // Build Search Tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTree(
      new pcl::search::KdTree<pcl::PointXYZ>());
  searchTree->setInputCloud(model_points_);
  searchTree_ = searchTree;
  ROS_INFO("[Done] Build up search tree\n");
}

void PreprocessModel::queryTree(pcl::PointXYZ p) {
  searchTree_->nearestKSearch(p, 1, nn_indices_, nn_dists_);
}

void PreprocessModel::addOutlier(pcl::PointXYZ p) {
  // idx_outliers_.push_back(i);
  meshing_points_->push_back(p);
}

// Source:https://github.com/tttamaki/ICP-test/blob/master/src/icp3_with_normal_iterative_view.cpp
void PreprocessModel::addNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                 pcl::PointCloud<pcl::Normal>::Ptr normals,
                                 int k) {
  pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  searchTree->setInputCloud(cloud);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
  normalEstimator.setInputCloud(cloud);
  normalEstimator.setSearchMethod(searchTree);
  normalEstimator.setKSearch(k);
  normalEstimator.compute(*normals);
}

void PreprocessModel::efficientRANSAC() {
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  this->addNormals(meshing_points_, normals, 15);

  std::vector<Point_with_normal> outliers(meshing_points_->size());
  for (unsigned i = 0; i < meshing_points_->size(); i++) {
    double x = (*meshing_points_)[i].x;
    double y = (*meshing_points_)[i].y;
    double z = (*meshing_points_)[i].z;

    double dx = (*normals)[i].normal_x;
    double dy = (*normals)[i].normal_y;
    double dz = (*normals)[i].normal_z;

    Kernel::Point_3 p_temp(x, y, z);
    Kernel::Vector_3 n_temp(dx, dy, dz);

    std::pair<Kernel::Point_3, Kernel::Vector_3> res(p_temp, n_temp);
    outliers[i] = res;
  }

  Efficient_ransac ransac;
  ransac.set_input(outliers);
  ransac.add_shape_factory<Plane>();
  // ransac.add_shape_factory<Cylinder>();

  Efficient_ransac::Parameters parameters;
  parameters.probability = 0.001;
  parameters.min_points = 200;
  parameters.epsilon = 0.03;
  parameters.cluster_epsilon = 0.1;  // 0.5
  parameters.normal_threshold = 0.95;

  ransac.detect(parameters);

  ROS_INFO("Detected Shapes: %d\n",
           ransac.shapes().end() - ransac.shapes().begin());
  ROS_INFO("Unassigned Points: %d\n", ransac.number_of_unassigned_points());

  pcl::PointIndices::Ptr detected_points(new pcl::PointIndices());
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  std::ofstream file;
  file.open("/home/philipp/Schreibtisch/outliers_ros.xyz", std::ofstream::app);
  Efficient_ransac::Shape_range shapes = ransac.shapes();
  Efficient_ransac::Shape_range::iterator it = shapes.begin();

  while (it != shapes.end()) {
    if (Plane* plane = dynamic_cast<Plane*>(it->get())) {
      const std::vector<std::size_t> idx_assigned_points =
          plane->indices_of_assigned_points();

      // TODO: Remove
      Kernel::Plane_3 plane_3 = static_cast<Kernel::Plane_3>(*plane);
      Eigen::Vector4d coeff_plane(plane_3.a(), plane_3.b(), plane_3.c(),
                                  plane_3.d());
      param_plane_.push_back(coeff_plane);

      Kernel::Vector_3 ransac_normal_temp = (*plane).plane_normal();
      Eigen::Vector3d ransac_normal(ransac_normal_temp.x(),
                                    ransac_normal_temp.y(),
                                    ransac_normal_temp.z());
      ransac_normal = ransac_normal.normalized();
      ransac_normals_.push_back(ransac_normal);

      Eigen::MatrixXd points(3, idx_assigned_points.size());
      Eigen::MatrixXd normals(3, idx_assigned_points.size());
      for (unsigned i = 0; i < idx_assigned_points.size(); i++) {
        detected_points->indices.push_back(idx_assigned_points.at(i));
        Point_with_normal point_with_normal = outliers[idx_assigned_points[i]];
        Kernel::Point_3 p = point_with_normal.first;
        Kernel::Vector_3 n = point_with_normal.second;
        file << p.x() << " " << p.y() << " " << p.z() << "\n";
        points.block<3, 1>(0, i) = Eigen::Vector3d(p.x(), p.y(), p.z());
        normals.block<3, 1>(0, i) = Eigen::Vector3d(n.x(), n.y(), n.z());
      }
      points_shape_.push_back(points);
      normals_shape_.push_back(normals);
      shape_id_.push_back(0);

    } /* else if (Cylinder* cyl = dynamic_cast<Cylinder*>(it->get())) {
       const std::vector<std::size_t> idx_assigned_points =
           cyl->indices_of_assigned_points();
       Eigen::MatrixXd points(3, idx_assigned_points.size());
       Eigen::MatrixXd normals(3, idx_assigned_points.size());
       for (unsigned i = 0; i < idx_assigned_points.size(); i++) {
         detected_points->indices.push_back(idx_assigned_points.at(i));
         Point_with_normal point_with_normal = outliers[idx_assigned_points[i]];
         Kernel::Point_3 p = point_with_normal.first;
         Kernel::Vector_3 n = point_with_normal.second;
         file << p.x() << " " << p.y() << " " << p.z() << "\n";
         points.block<3, 1>(0, i) = Eigen::Vector3d(p.x(), p.y(), p.z());
         normals.block<3, 1>(0, i) = Eigen::Vector3d(n.x(), n.y(), n.z());
       }
       points_shape_.push_back(points);
       normals_shape_.push_back(points);
       shape_id_.push_back(1);
     }*/
    it++;
  }
  file.close();

  extract.setInputCloud(meshing_points_);
  extract.setIndices(detected_points);
  extract.setNegative(true);
  extract.filter(*meshing_points_);
}

// Source:
// https://github.com/apalomer/plane_fitter/blob/master/src/check_planarity.cpp
// and
// https://pointclouds.org/documentation/tutorials/cylinder_segmentation.html
void PreprocessModel::SACSegmentation() {
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.03);
  seg.setMaxIterations(1000);

  int i = 0, nr_points = (int)meshing_points_->points.size();
  while (meshing_points_->points.size() > 0.3 * nr_points) {
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
        new pcl::PointCloud<pcl::Normal>);
    this->addNormals(meshing_points_, cloud_normals, 100);
    seg.setInputNormals(cloud_normals);
    seg.setInputCloud(meshing_points_);
    seg.segment(*inliers, *coefficients);

    int nr_inliers = inliers->indices.size();
    Eigen::MatrixXd points(3, nr_inliers);
    Eigen::MatrixXd normals(3, nr_inliers);
    double error_sum = 0;
    for (unsigned i = 0; i < nr_inliers; i++) {
      pcl::PointXYZ p = meshing_points_->points[inliers->indices[i]];
      pcl::Normal n = cloud_normals->points[inliers->indices[i]];
      points.block<3, 1>(0, i) = Eigen::Vector3d(p.x, p.y, p.z);
      normals.block<3, 1>(0, i) =
          Eigen::Vector3d(n.normal[0], n.normal[1], n.normal[2]);

      double f1 =
          fabs(coefficients->values[0] * p.x + coefficients->values[1] * p.y +
               coefficients->values[2] * p.z + coefficients->values[3]);
      double f2 = sqrt(pow(coefficients->values[0], 2) +
                       pow(coefficients->values[1], 2) +
                       pow(coefficients->values[2], 2));
      error_sum += f1 / f2;
    }
    double mean_error = error_sum / nr_inliers;

    ROS_INFO("Mean error: %f\n", mean_error);
    if (mean_error < 0.03) {
      points_shape_.push_back(points);
      normals_shape_.push_back(normals);
      shape_id_.push_back(0);

      Eigen::Vector3d plane_normal(coefficients->values[0],
                                   coefficients->values[1],
                                   coefficients->values[2]);
      plane_normal = plane_normal.normalized();
      ransac_normals_.push_back(plane_normal);
    }

    extract.setInputCloud(meshing_points_);
    extract.setIndices(inliers);
    extract.setNegative(true);
    pcl::PointCloud<pcl::PointXYZ> cloudF;
    extract.filter(*meshing_points_);
  }
}

void PreprocessModel::applyFilter() {
  ROS_INFO("Size before filtering: %d\n", meshing_points_->size());

  pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ> octree_filter(
      0.01f);
  octree_filter.setInputCloud(meshing_points_);
  octree_filter.addPointsFromInputCloud();
  pcl::PointCloud<pcl::PointXYZ>::VectorType voxelCentroids;
  octree_filter.getVoxelCentroids(voxelCentroids);
  meshing_points_->clear();
  for (int i = 0; i < voxelCentroids.size(); i++) {
    meshing_points_->push_back(voxelCentroids[i]);
  }

  /*
  Error for too big point clouds
  [pcl::VoxelGrid::applyFilter] Leaf size is too small for the input dataset.
  Integer indices would overflow.[ pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setInputCloud(meshing_points_);
  voxel_grid.setLeafSize(0.01f, 0.01f, 0.01f);
  voxel_grid.filter(*meshing_points_);
  */
  ROS_INFO("Size after filtering: %d\n", meshing_points_->size());
};

std::vector<Eigen::MatrixXd>* PreprocessModel::getPointShapes() {
  return &points_shape_;
}

std::vector<Eigen::MatrixXd>* PreprocessModel::getNormalShapes() {
  return &normals_shape_;
}

std::vector<Eigen::Vector3d>* PreprocessModel::getRansacNormals() {
  return &ransac_normals_;
}

void PreprocessModel::clearBuffer() { meshing_points_->clear(); }

std::vector<int>* PreprocessModel::getShapeIDs() { return &shape_id_; }

int PreprocessModel::getOutlierCount() { return meshing_points_->size(); }

float PreprocessModel::getMinDistance() { return nn_dists_[0]; }

int PreprocessModel::getIndex() { return nn_indices_[0]; }

void PreprocessModel::printOutliers() {
  for (unsigned i = 0; i < idx_outliers_.size(); i++) {
    ROS_INFO("Outlier idx: %d\n", idx_outliers_.at(i));
  }
}

void PreprocessModel::clearRansacShapes() {
  points_shape_.clear();
  normals_shape_.clear();
  ransac_normals_.clear();
  shape_id_.clear();
}

}  // namespace cpt_reconstruction
}  // namespace cad_percept

/*
Using Polygonal_surface_reconstruction
TODO: Too long processing - Gruobi solver?
//#define CGAL_USE_GLPK
#define CGAL_USE_SCIP
//#include <CGAL/GLPK_mixed_integer_program_traits.h>
//typedef CGAL::GLPK_mixed_integer_program_traits<double> MIP_Solver;

#include <CGAL/SCIP_mixed_integer_program_traits.h>
typedef CGAL::SCIP_mixed_integer_program_traits<double> MIP_Solver;

typedef Kernel::Point_3 Point2;
typedef Kernel::Vector_3  Vector2;
// Point with normal, and plane index
typedef boost::tuple<Point2, Vector2, int>  PNI2;
typedef std::vector<PNI2>  Point_vector2;
typedef CGAL::Nth_of_tuple_property_map<0, PNI2> Point_map2;
typedef CGAL::Nth_of_tuple_property_map<1, PNI2> Normal_map2;
typedef CGAL::Nth_of_tuple_property_map<2, PNI2> Plane_index_map2;
typedef CGAL::Shape_detection::Efficient_RANSAC_traits<Kernel, Point_vector2,
Point_map2, Normal_map2> Traits2; typedef
CGAL::Shape_detection::Efficient_RANSAC<Traits2> Efficient_ransac2; typedef
CGAL::Shape_detection::Plane<Traits2>  Plane2; typedef
CGAL::Shape_detection::Point_to_shape_index_map<Traits2>
Point_to_shape_index_map2; typedef
CGAL::Polygonal_surface_reconstruction<Kernel>
Polygonal_surface_reconstruction2; typedef CGAL::Surface_mesh<Point2>
Surface_mesh2; std::vector<boost::tuple<Kernel::Point_3, Kernel::Vector_3, int>>
points_sur; for (int i = 0; i < outliers.size(); i++){
    boost::tuple<Kernel::Point_3, Kernel::Vector_3, int> tmp(outliers[i].first,
outliers[i].second, 0); points_sur.push_back(tmp);
  }

  Efficient_ransac2 ransac;
  ransac.set_input(points_sur);
  ransac.add_shape_factory<Plane2>();

  Efficient_ransac2::Parameters parameters;
  parameters.probability = 0.005;
  parameters.min_points = 250;
  parameters.epsilon = 0.03;
  parameters.cluster_epsilon = 0.5;
  parameters.normal_threshold = 0.9;

  ransac.detect(parameters);

  ROS_INFO("Detected Shapes: %d\n",
           ransac.shapes().end() - ransac.shapes().begin());
  ROS_INFO("Unassigned Points: %d\n", ransac.number_of_unassigned_points());

  Efficient_ransac2::Plane_range planes_sur = ransac.planes();
  std::size_t num_planes = planes_sur.size();

  Point_to_shape_index_map2 shape_index_map(points_sur, planes_sur);

  for (std::size_t i = 0; i < points_sur.size(); ++i) {
    int plane_index = get(shape_index_map, i);
    points_sur[i].get<2>() = plane_index;
  }

  Polygonal_surface_reconstruction2 algo(
          points_sur,
          Point_map2(),
          Normal_map2(),
          Plane_index_map2()
  );

  Surface_mesh2 model;
  algo.reconstruct<MIP_Solver>(model);
  std::string output_file = "/home/philipp/Schreibtisch/Meshes/mesh_" +
                       std::to_string(num_planes) + ".off";
  std::ofstream output_stream(output_file.c_str());
  CGAL::write_off(output_stream, model);
*/