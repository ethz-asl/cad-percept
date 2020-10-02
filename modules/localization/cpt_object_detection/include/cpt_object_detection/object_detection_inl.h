#ifndef CPT_OBJECT_DETECTION_MODULES_LOCALIZATION_CPT_OBJECT_DETECTION_INCLUDE_CPT_OBJECT_DETECTION_OBJECT_DETECTION_INL_H_
#define CPT_OBJECT_DETECTION_MODULES_LOCALIZATION_CPT_OBJECT_DETECTION_INCLUDE_CPT_OBJECT_DETECTION_OBJECT_DETECTION_INL_H_

#include <modelify/registration_toolbox/fast_global_registration.h>
#include <teaser/registration.h>

namespace cad_percept {
namespace object_detection {

template <typename descriptor_type>
Transformation computeTransformUsing3dFeatures(
    MatchingMethod matching_method, const modelify::PointSurfelCloudType::Ptr& detection_surfels,
    const modelify::PointSurfelCloudType::Ptr& detection_keypoints,
    const typename pcl::PointCloud<descriptor_type>::Ptr& detection_descriptors,
    const modelify::PointSurfelCloudType::Ptr& object_surfels,
    const modelify::PointSurfelCloudType::Ptr& object_keypoints,
    const typename pcl::PointCloud<descriptor_type>::Ptr& object_descriptors,
    double similarity_threshold, const modelify::CorrespondencesTypePtr& correspondences) {
  if (object_descriptors->empty()) {
    LOG(ERROR) << "Object pointcloud has no features!";
    return Transformation();
  }

  if (detection_descriptors->empty()) {
    LOG(ERROR) << "Detection pointcloud has no features!";
    return Transformation();
  }

  // Match features
  switch (matching_method) {
    case kConventional:
      return computeTransformUsingModelify<descriptor_type>(
          detection_keypoints, detection_descriptors, object_keypoints, object_descriptors,
          similarity_threshold, correspondences);
      break;
    case kFastGlobalRegistration:
      return computeTransformUsingFgr<descriptor_type>(
          detection_surfels, detection_keypoints, detection_descriptors, object_surfels,
          object_keypoints, object_descriptors, correspondences);
      break;
    case kTeaser:
      return computeTransformUsingTeaser<descriptor_type>(
          detection_keypoints, detection_descriptors, object_keypoints, object_descriptors,
          similarity_threshold, correspondences);
      break;
    default:
      LOG(ERROR) << "Unknown matching method! " << matching_method;
      return Transformation();
  }
}

template <typename descriptor_type>
Transformation computeTransformUsingFgr(
    const modelify::PointSurfelCloudType::Ptr& detection_surfels,
    const modelify::PointSurfelCloudType::Ptr& detection_keypoints,
    const typename pcl::PointCloud<descriptor_type>::Ptr& detection_descriptors,
    const modelify::PointSurfelCloudType::Ptr& object_surfels,
    const modelify::PointSurfelCloudType::Ptr& object_keypoints,
    const typename pcl::PointCloud<descriptor_type>::Ptr& object_descriptors,
    const modelify::CorrespondencesTypePtr& correspondences) {
  std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
  modelify::registration_toolbox::FastGlobalRegistrationParams fgr_params;
  fgr_params.crosscheck_test = true;  // 300 -> 30
  fgr_params.tuple_test = false;      // 30 -> 0
  fgr_params.refine_using_icp = false;
  fgr_params.use_absolute_scale = true;
  modelify::Transformation transform;
  modelify::Correspondences corrs;
  if (!modelify::registration_toolbox::fast_global_registration::
          estimateTransformationFastGlobalRegistration<modelify::PointSurfelType, descriptor_type>(
              detection_surfels, object_surfels, detection_keypoints, object_keypoints,
              detection_descriptors, object_descriptors, fgr_params, &corrs, &transform)) {
    LOG(ERROR) << "Fast global registration was not successful!";
    return Transformation();
  }
  LOG(INFO) << "Time FGR: " << (std::chrono::steady_clock::now() - start).count();

  for (const modelify::CorrespondencePair& correspondence : corrs) {
    pcl::Correspondence corr;
    corr.index_match = correspondence.first;
    corr.index_query = correspondence.second;
    correspondences->push_back(corr);
  }

  return Transformation(transform);
}

template <typename descriptor_type>
Transformation computeTransformUsingModelify(
    const modelify::PointSurfelCloudType::Ptr& detection_keypoints,
    const typename pcl::PointCloud<descriptor_type>::Ptr& detection_descriptors,
    const modelify::PointSurfelCloudType::Ptr& object_keypoints,
    const typename pcl::PointCloud<descriptor_type>::Ptr& object_descriptors,
    double correspondence_threshold, const modelify::CorrespondencesTypePtr& correspondences) {
  // Find correspondences
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  modelify::registration_toolbox::FlannSearchMatchingParams flann_params;
  flann_params.num_of_correspondences = 3;
  if (object_descriptors->size() < flann_params.num_of_correspondences ||
      detection_descriptors->size() < flann_params.num_of_correspondences) {
    LOG(ERROR) << "Too few features found! Object: " << object_descriptors->size() << "/"
               << flann_params.num_of_correspondences
               << ", Detection: " << detection_descriptors->size() << "/"
               << flann_params.num_of_correspondences;
    return Transformation();
  }
  flann_params.similarity_threshold = correspondence_threshold;
  if (correspondence_threshold == 0) {
    flann_params.prefilter = false;
  }
  flann_params.keep_statistics = true;
  modelify::registration_toolbox::matchDescriptorsFlannSearch<descriptor_type>(
      detection_descriptors, object_descriptors, flann_params, correspondences);
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  LOG(INFO) << "Time correspondences: "
            << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() / 1e6;
  if (correspondences->empty()) {
    LOG(ERROR) << "No correspondences found!";
    return Transformation();
  }
  LOG(INFO) << "Found " << correspondences->size() << " correspondences.";

  // Get transformation between detection and object pointcloud
  return computeTransformFromCorrespondences(detection_keypoints, object_keypoints,
                                             correspondences);
}

template <typename descriptor_type>
Transformation computeTransformUsingTeaser(
    const modelify::PointSurfelCloudType::Ptr& detection_keypoints,
    const typename pcl::PointCloud<descriptor_type>::Ptr& detection_descriptors,
    const modelify::PointSurfelCloudType::Ptr& object_keypoints,
    const typename pcl::PointCloud<descriptor_type>::Ptr& object_descriptors,
    double correspondence_threshold, const modelify::CorrespondencesTypePtr& correspondences) {
  // Find correspondences
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  modelify::registration_toolbox::FlannSearchMatchingParams flann_params;
  flann_params.num_of_correspondences = 3;
  if (object_descriptors->size() < flann_params.num_of_correspondences ||
      detection_descriptors->size() < flann_params.num_of_correspondences) {
    LOG(ERROR) << "Too few features found! Object: " << object_descriptors->size() << "/"
               << flann_params.num_of_correspondences
               << ", Detection: " << detection_descriptors->size() << "/"
               << flann_params.num_of_correspondences;
    return Transformation();
  }
  flann_params.similarity_threshold = correspondence_threshold;
  if (correspondence_threshold == 0) {
    flann_params.prefilter = false;
  }
  flann_params.keep_statistics = true;
  modelify::registration_toolbox::matchDescriptorsFlannSearch<descriptor_type>(
      detection_descriptors, object_descriptors, flann_params, correspondences);
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  LOG(INFO) << "Time correspondences: "
            << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() / 1e6;
  if (correspondences->empty()) {
    LOG(ERROR) << "No correspondences found!";
    return Transformation();
  }
  LOG(INFO) << "Found " << correspondences->size() << " correspondences.";

  // Convert keypoints into matrix
  begin = std::chrono::steady_clock::now();
  Eigen::Matrix<double, 3, Eigen::Dynamic> object_matrix(3, correspondences->size());
  Eigen::Matrix<double, 3, Eigen::Dynamic> detection_matrix(3, correspondences->size());
  uint idx = 0;
  for (const auto& correspondence : *correspondences) {
    object_matrix.col(idx) << object_keypoints->points[correspondence.index_match].x,
        detection_keypoints->points[correspondence.index_match].y,
        detection_keypoints->points[correspondence.index_match].z;
    detection_matrix.col(idx) << detection_keypoints->points[correspondence.index_query].x,
        detection_keypoints->points[correspondence.index_query].y,
        detection_keypoints->points[correspondence.index_query].z;
    ++idx;
  }
  end = std::chrono::steady_clock::now();
  LOG(INFO) << "Time conversion: "
            << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() / 1e6;

  // Solve with TEASER++
  teaser::RobustRegistrationSolver::Params params;
  params.estimate_scaling = false;
  params.rotation_cost_threshold = 0.005;
  teaser::RobustRegistrationSolver solver(params);
  begin = std::chrono::steady_clock::now();
  teaser::RegistrationSolution solution = solver.solve(detection_matrix, object_matrix);
  end = std::chrono::steady_clock::now();
  LOG(INFO) << "Time teaser: "
            << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() / 1e6;

  if (!solution.valid) {
    LOG(ERROR) << "Registration using Teaser failed!";
    return Transformation();
  }

  Eigen::Matrix3f rotation_matrix = solution.rotation.cast<float>();
  Transformation T_teaser =
      Transformation(solution.translation.cast<float>(), Quaternion(rotation_matrix));
  LOG(INFO) << "Transformation teaser:\n" << T_teaser.getTransformationMatrix();
  return T_teaser;
}

template <typename descriptor_type>
bool get3dFeatures(const KeypointType& keypoint_type,
                   const pcl::PointCloud<pcl::PointXYZ>& pointcloud_xyz,
                   const modelify::PointSurfelCloudType::Ptr& pointcloud_surfel_ptr,
                   const modelify::PointSurfelCloudType::Ptr& keypoints,
                   const typename pcl::PointCloud<descriptor_type>::Ptr& descriptors) {
  std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
  // Get surfels
  pcl::NormalEstimation<pcl::PointXYZ, modelify::PointSurfelType> normal_estimator;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr pcl_ptr =
      boost::make_shared<const pcl::PointCloud<pcl::PointXYZ>>(pointcloud_xyz);
  normal_estimator.setInputCloud(pcl_ptr);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZ>());
  normal_estimator.setSearchMethod(kd_tree);
  // TODO(gasserl): smarter param?
  //  constexpr double search_radius = 0.1;
  //  normal_estimator.setRadiusSearch(search_radius);
  constexpr int k_radius = 4;
  normal_estimator.setKSearch(k_radius);
  normal_estimator.compute(*pointcloud_surfel_ptr);
  for (size_t i = 0; i < pointcloud_xyz.size(); ++i) {
    pointcloud_surfel_ptr->points[i].x = pointcloud_xyz.points[i].x;
    pointcloud_surfel_ptr->points[i].y = pointcloud_xyz.points[i].y;
    pointcloud_surfel_ptr->points[i].z = pointcloud_xyz.points[i].z;
  }

  size_t size_before = pointcloud_surfel_ptr->size();
  size_t i = 0;
  while (i < pointcloud_surfel_ptr->size()) {
    if (std::isnan(pointcloud_surfel_ptr->points[i].normal_x) ||
        std::isnan(pointcloud_surfel_ptr->points[i].normal_y) ||
        std::isnan(pointcloud_surfel_ptr->points[i].normal_z) ||
        std::isnan(pointcloud_surfel_ptr->points[i].x) ||
        std::isnan(pointcloud_surfel_ptr->points[i].y) ||
        std::isnan(pointcloud_surfel_ptr->points[i].z)) {
      pointcloud_surfel_ptr->points.erase(pointcloud_surfel_ptr->points.begin() + i);
    } else {
      ++i;
    }
  }
  if (size_before > pointcloud_surfel_ptr->size()) {
    LOG(INFO) << "Filtered " << size_before - pointcloud_surfel_ptr->size()
              << " points with NaN points or normals";
  }
  LOG(INFO) << "Time normals: " << (std::chrono::steady_clock::now() - start).count();

  std::chrono::steady_clock::time_point start_keypoints = std::chrono::steady_clock::now();
  if (!getKeypoints(keypoint_type, pointcloud_surfel_ptr, keypoints)) {
    return false;
  }
  LOG(INFO) << "Time keypoints: " << (std::chrono::steady_clock::now() - start_keypoints).count();

  std::chrono::steady_clock::time_point start_descriptors = std::chrono::steady_clock::now();
  getDescriptors<descriptor_type>(pointcloud_surfel_ptr, keypoints, descriptors);
  LOG(INFO) << "Time descriptors: "
            << (std::chrono::steady_clock::now() - start_descriptors).count();
  LOG(INFO) << "Time 3D features: " << (std::chrono::steady_clock::now() - start).count();
  return true;
}

}  // namespace object_detection
}  // namespace cad_percept
#endif  // CPT_OBJECT_DETECTION_MODULES_LOCALIZATION_CPT_OBJECT_DETECTION_INCLUDE_CPT_OBJECT_DETECTION_OBJECT_DETECTION_INL_H_
