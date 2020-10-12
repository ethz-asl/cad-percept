#include "test_matcher/3d_descriptor_matcher.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

namespace cad_percept {
namespace matching_algorithms {

StruDe::StruDe()
    : map_computed_(false),
      keypoints_map_(new pcl::PointCloud<pcl::PointSurfel>()),
      descriptors_map_(new pcl::PointCloud<pcl::SHOT352>) {}
void StruDe::strudeMatch(Eigen::Matrix4d &res_transform,
                         const pcl::PointCloud<pcl::PointXYZ> &lidar_scan,
                         const pcl::PointCloud<pcl::PointXYZ> &sampled_map) {
  std::cout << "///////////////////////////////////////////////" << std::endl;
  std::cout << "     Structural Descriptors matcher started    " << std::endl;
  std::cout << "///////////////////////////////////////////////" << std::endl;

  ros::NodeHandle nh_private("~");
  std::string downsample_points = nh_private.param<std::string>("GoICPdownsample", "1000");
  std::string goicp_location = nh_private.param<std::string>("goicp_folder", "fail");

  pcl::PointCloud<pcl::PointSurfel>::Ptr strude_lidar(new pcl::PointCloud<pcl::PointSurfel>);
  pcl::copyPointCloud(lidar_scan, *strude_lidar);
  pcl::PointCloud<pcl::PointSurfel>::Ptr strude_map(new pcl::PointCloud<pcl::PointSurfel>);
  pcl::copyPointCloud(sampled_map, *strude_map);

  // Preprocess data
  //  pcl::PCLPointCloud2::Ptr lidar_scan_pc2(new pcl::PCLPointCloud2);
  //  pcl::toPCLPointCloud2(*lidar_scan, *lidar_scan_pc2);
  //  pcl::PCLPointCloud2::Ptr sampled_map_pc2(new pcl::PCLPointCloud2);
  //  pcl::toPCLPointCloud2(*sampled_map, *sampled_map_pc2);
  //
  //  // Fitler map with voxel grid filter to equalize density.
  //  pcl::VoxelGrid<pcl::PCLPointCloud2> filter;
  //  filter.setLeafSize (0.1f, 0.1f, 0.1f);
  //  filter.setInputCloud (sampled_map_pc2);
  //  filter.filter (*sampled_map_pc2);
  //  filter.setInputCloud (lidar_scan_pc2);
  //  filter.filter (*lidar_scan_pc2);

  // Compute Normals on pointcloud.
  //  pcl::PointCloud<pcl::PointXYZ>::Ptr surfel_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  //  pcl::copyPointCloud(*filtered_cloud, *surfel_filtered);

  pcl::NormalEstimation<pcl::PointSurfel, pcl::PointSurfel> ne;
  ne.setKSearch(10);
  ne.setInputCloud(strude_lidar);
  ne.compute(*strude_lidar);

  ne.setInputCloud(strude_map);
  ne.compute(*strude_map);
  // Detect ISS keypoints in data.

  typename pcl::search::KdTree<pcl::PointSurfel>::Ptr tree(
      new pcl::search::KdTree<pcl::PointSurfel>());
  pcl::PointCloud<pcl::PointSurfel>::Ptr keypoints_lidar(new pcl::PointCloud<pcl::PointSurfel>());
  //  pcl::PointCloud<pcl::PointSurfel>::Ptr keypoints_map(new pcl::PointCloud<pcl::PointSurfel>());
  double model_resolution = 0.2;
  pcl::ISSKeypoint3D<pcl::PointSurfel, pcl::PointSurfel> iss_detector;
  iss_detector.setSearchMethod(tree);
  iss_detector.setSalientRadius(6 * model_resolution);
  iss_detector.setNonMaxRadius(4 * model_resolution);
  iss_detector.setThreshold21(0.975);
  iss_detector.setThreshold32(0.975);
  iss_detector.setMinNeighbors(5);
  iss_detector.setNumberOfThreads(4);
  iss_detector.setInputCloud(strude_lidar);
  iss_detector.compute(*keypoints_lidar);
  if (!map_computed_) {
    iss_detector.setInputCloud(strude_map);
    iss_detector.compute(*keypoints_map_);
  }

  // Compute Descriptors at keypoints (SHOT)
  pcl::SHOTEstimation<pcl::PointSurfel, pcl::PointSurfel, pcl::SHOT352> shotEstimation;
  shotEstimation.setSearchMethod(tree);
  //  shotEstimation.setKSearch(10);
  shotEstimation.setRadiusSearch(2.0);
  shotEstimation.setInputCloud(keypoints_lidar);
  shotEstimation.setSearchSurface(strude_lidar);
  shotEstimation.setInputNormals(strude_lidar);
  pcl::PointCloud<pcl::SHOT352>::Ptr descriptors_lidar(new pcl::PointCloud<pcl::SHOT352>);
  shotEstimation.compute(*descriptors_lidar);
  std::cout << __LINE__ << __FILE__ << std::endl;
  if (!map_computed_) {
    shotEstimation.setInputCloud(keypoints_map_);
    shotEstimation.setSearchSurface(strude_map);
    shotEstimation.setInputNormals(strude_map);
    //    pcl::PointCloud<pcl::SHOT352>::Ptr descriptors_map(new pcl::PointCloud<pcl::SHOT352>);
    shotEstimation.compute(*descriptors_map_);
  } else {
    std::cout << "Skipping map features." << std::endl;
  }

  std::cout << __LINE__ << __FILE__ << std::endl;
  // Calculate matches between data
  pcl::registration::CorrespondenceEstimation<pcl::SHOT352, pcl::SHOT352> est;
  pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
  est.setInputSource(descriptors_lidar);
  est.setInputTarget(descriptors_map_);
  est.determineCorrespondences(*correspondences);
  std::cout << __LINE__ << __FILE__ << std::endl;

  // Perform correspondence clustering
  pcl::GeometricConsistencyGrouping<pcl::PointSurfel, pcl::PointSurfel> grouping;
  std::vector<size_t> vote_size;
  std::vector<size_t> unsorted_vote_size;

  grouping.setSceneCloud(keypoints_map_);
  grouping.setInputCloud(keypoints_lidar);
  grouping.setModelSceneCorrespondences(correspondences);
  grouping.setGCThreshold(6.0);
  grouping.setGCSize(3.0);

  std::cout << __LINE__ << __FILE__ << std::endl;
  TransformationVector transforms;
  std::vector<pcl::Correspondences> clustered_correspondences;
  grouping.recognize(transforms, clustered_correspondences);

  std::cout << __LINE__ << __FILE__ << std::endl;
  std::cout << "size: " << transforms.size() << std::endl;
  std::cout << "size: " << clustered_correspondences.size() << std::endl;

  if (transforms.size() == 0) {
    std::cout << "Cannot find transformation." << std::endl;
    res_transform = Eigen::Matrix4d::Identity();
  } else {
    res_transform = transforms[0].cast<double>();
  }
  map_computed_ = true;
}

}  // namespace matching_algorithms
}  // namespace cad_percept
