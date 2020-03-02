// Based on tutorial of pointcloud.org correspondence_grouping
#include <pcl/correspondence.h>
#include <pcl/features/board.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <Eigen/Geometry>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include "test_Matcher/test_Matcher.h"

typedef pcl::PointXYZRGBA PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

// Algorithm params
bool use_hough_(true);
float model_ss_(0.01f);
float scene_ss_(0.03f);
float rf_rad_(0.15f);
float descr_rad_(0.15f);
float cg_size_(0.1f);
float cg_thresh_(5.0f);

namespace cad_percept {
namespace matching_algorithms {

void test_Matcher::match(float (&transformTR)[6]) {
  std::cout << "Feature matcher started" << std::endl;
  pcl::PointCloud<PointType>::Ptr model(new pcl::PointCloud<PointType>());
  copyPointCloud(lidar_frame, *model);
  pcl::PointCloud<PointType>::Ptr scene(new pcl::PointCloud<PointType>());
  copyPointCloud(sample_map, *scene);

  // Compute Normals
  pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
  pcl::PointCloud<NormalType>::Ptr model_normals(new pcl::PointCloud<NormalType>());
  pcl::PointCloud<NormalType>::Ptr scene_normals(new pcl::PointCloud<NormalType>());

  norm_est.setKSearch(10);
  norm_est.setInputCloud(model);
  norm_est.compute(*model_normals);

  norm_est.setInputCloud(scene);
  norm_est.compute(*scene_normals);

  //  Downsample Clouds to Extract keypoints
  pcl::PointCloud<PointType>::Ptr model_keypoints(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr scene_keypoints(new pcl::PointCloud<PointType>());

  pcl::UniformSampling<PointType> uniform_sampling;
  uniform_sampling.setInputCloud(model);
  uniform_sampling.setRadiusSearch(model_ss_);
  uniform_sampling.filter(*model_keypoints);
  std::cout << "Model total points: " << model->size()
            << "; Selected Keypoints: " << model_keypoints->size() << std::endl;

  uniform_sampling.setInputCloud(scene);
  uniform_sampling.setRadiusSearch(scene_ss_);
  uniform_sampling.filter(*scene_keypoints);
  std::cout << "Scene total points: " << scene->size()
            << "; Selected Keypoints: " << scene_keypoints->size() << std::endl;

  // Compute Descriptor for keypoints
  pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
  descr_est.setRadiusSearch(descr_rad_);

  pcl::PointCloud<DescriptorType>::Ptr model_descriptors(new pcl::PointCloud<DescriptorType>());
  descr_est.setInputCloud(model_keypoints);
  descr_est.setInputNormals(model_normals);
  descr_est.setSearchSurface(model);
  descr_est.compute(*model_descriptors);

  pcl::PointCloud<DescriptorType>::Ptr scene_descriptors(new pcl::PointCloud<DescriptorType>());
  descr_est.setInputCloud(scene_keypoints);
  descr_est.setInputNormals(scene_normals);
  descr_est.setSearchSurface(scene);
  descr_est.compute(*scene_descriptors);

  //  Find Model-Scene Correspondences with KdTree
  pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());

  pcl::KdTreeFLANN<DescriptorType> match_search;
  match_search.setInputCloud(model_descriptors);

  //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor
  //  cloud and add it to the correspondences vector.
  for (std::size_t i = 0; i < scene_descriptors->size(); ++i) {
    std::vector<int> neigh_indices(1);
    std::vector<float> neigh_sqr_dists(1);
    if (!std::isfinite(scene_descriptors->at(i).descriptor[0]))  // skipping NaNs
    {
      continue;
    }
    int found_neighs =
        match_search.nearestKSearch(scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);
    if (found_neighs == 1 &&
        neigh_sqr_dists[0] <
            0.25f)  //  add match only if the squared descriptor distance is less than 0.25 (SHOT
                    //  descriptor distances are between 0 and 1 by design)
    {
      pcl::Correspondence corr(neigh_indices[0], static_cast<int>(i), neigh_sqr_dists[0]);
      model_scene_corrs->push_back(corr);
    }
  }
  std::cout << "Correspondences found: " << model_scene_corrs->size() << std::endl;

  //  Actual Clustering

  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
  std::vector<pcl::Correspondences> clustered_corrs;
  //  Using Hough3D
  if (use_hough_) {
    //
    //  Compute (Keypoints) Reference Frames only for Hough
    //
    pcl::PointCloud<RFType>::Ptr model_rf(new pcl::PointCloud<RFType>());
    pcl::PointCloud<RFType>::Ptr scene_rf(new pcl::PointCloud<RFType>());

    pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
    rf_est.setFindHoles(true);
    rf_est.setRadiusSearch(rf_rad_);

    rf_est.setInputCloud(model_keypoints);
    rf_est.setInputNormals(model_normals);
    rf_est.setSearchSurface(model);
    rf_est.compute(*model_rf);

    rf_est.setInputCloud(scene_keypoints);
    rf_est.setInputNormals(scene_normals);
    rf_est.setSearchSurface(scene);
    rf_est.compute(*scene_rf);

    //  Clustering
    pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
    clusterer.setHoughBinSize(cg_size_);
    clusterer.setHoughThreshold(cg_thresh_);
    clusterer.setUseInterpolation(true);
    clusterer.setUseDistanceWeight(false);

    clusterer.setInputCloud(model_keypoints);
    clusterer.setInputRf(model_rf);
    clusterer.setSceneCloud(scene_keypoints);
    clusterer.setSceneRf(scene_rf);
    clusterer.setModelSceneCorrespondences(model_scene_corrs);

    // clusterer.cluster (clustered_corrs);
    if (!clusterer.recognize(rototranslations, clustered_corrs)) {
      std::cout << "An error occured" << std::endl;
    }
  } else  // Using GeometricConsistency
  {
    pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
    gc_clusterer.setGCSize(cg_size_);
    gc_clusterer.setGCThreshold(cg_thresh_);

    gc_clusterer.setInputCloud(model_keypoints);
    gc_clusterer.setSceneCloud(scene_keypoints);
    gc_clusterer.setModelSceneCorrespondences(model_scene_corrs);

    // gc_clusterer.cluster (clustered_corrs);
    if (!gc_clusterer.recognize(rototranslations, clustered_corrs)) {
      std::cout << "An error occured" << std::endl;
    }
  }

  Eigen::Matrix3f rotation = rototranslations[0].block<3, 3>(0, 0);
  Eigen::Vector3f euler_angle = rotation.eulerAngles(0, 1, 2);
  Eigen::Vector3f translation = rototranslations[0].block<3, 1>(0, 3);

  transformTR[0] = translation[0];
  transformTR[1] = translation[1];
  transformTR[2] = translation[2];
  transformTR[3] = euler_angle[0];
  transformTR[4] = euler_angle[1];
  transformTR[5] = euler_angle[2];
}
}  // namespace matching_algorithms
}  // namespace cad_percept
