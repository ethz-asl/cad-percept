#ifndef CPT_OBJECT_DETECTION_OBJECT_DETECTION_INL_H_
#define CPT_OBJECT_DETECTION_OBJECT_DETECTION_INL_H_

#include <modelify/registration_toolbox/fast_global_registration.h>
#include <teaser/registration.h>

namespace cad_percept::object_detection {

template <typename descriptor_type>
Transformation computeTransformUsing3dFeatures(
    MatchingMethod matching_method, const modelify::PointSurfelCloudType::Ptr& detection_surfels,
    const modelify::PointSurfelCloudType::Ptr& detection_keypoints,
    const typename pcl::PointCloud<descriptor_type>::Ptr& detection_descriptors,
    const modelify::PointSurfelCloudType::Ptr& object_surfels,
    const modelify::PointSurfelCloudType::Ptr& object_keypoints,
    const typename pcl::PointCloud<descriptor_type>::Ptr& object_descriptors,
    double similarity_threshold, const modelify::CorrespondencesTypePtr& correspondences) {
  CHECK(detection_surfels);
  CHECK(detection_keypoints);
  CHECK(detection_descriptors);
  CHECK(object_surfels);
  CHECK(object_keypoints);
  CHECK(object_descriptors);
  CHECK(correspondences);

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
    case kGeometricConsistency:
      return computeTransformUsingGeometricConsistency<descriptor_type>(
          detection_keypoints, detection_descriptors, object_keypoints, object_descriptors,
          similarity_threshold, correspondences);
    case kFastGlobalRegistration:
      return computeTransformUsingFgr<descriptor_type>(
          detection_surfels, detection_keypoints, detection_descriptors, object_surfels,
          object_keypoints, object_descriptors, correspondences);
    case kTeaser:
      return computeTransformUsingTeaser<descriptor_type>(
          detection_keypoints, detection_descriptors, object_keypoints, object_descriptors,
          similarity_threshold, correspondences);
    default:
      LOG(ERROR) << "Unknown matching method! " << matching_method;
      return Transformation();
  }
}

template <typename descriptor_type>
modelify::CorrespondencesType computeCorrespondences(
    const modelify::PointSurfelCloudType::Ptr& detection_keypoints,
    const typename pcl::PointCloud<descriptor_type>::Ptr& detection_descriptors,
    const modelify::PointSurfelCloudType::Ptr& object_keypoints,
    const typename pcl::PointCloud<descriptor_type>::Ptr& object_descriptors,
    double similarity_threshold) {
  CHECK(detection_keypoints);
  CHECK(detection_descriptors);
  CHECK(object_keypoints);
  CHECK(object_descriptors);

  modelify::CorrespondencesTypePtr correspondences(new modelify::CorrespondencesType());

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
    return *correspondences;
  }
  flann_params.similarity_threshold = similarity_threshold;
  if (similarity_threshold == 0) {
    flann_params.prefilter = false;
  }
  flann_params.keep_statistics = true;
  modelify::registration_toolbox::matchDescriptorsFlannSearch<descriptor_type>(
      object_descriptors, detection_descriptors, flann_params, correspondences);
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  if (correspondences->empty()) {
    LOG(ERROR) << "No correspondences found!";
    return *correspondences;
  }
  LOG(INFO) << "Found " << correspondences->size() << " correspondences.";
  LOG(INFO) << "Time correspondences: " << std::chrono::duration<float>(end - begin).count()
            << " s";
  return *correspondences;
}

template <typename descriptor_type>
Transformation computeTransformUsingGeometricConsistency(
    const modelify::PointSurfelCloudType::Ptr& detection_keypoints,
    const typename pcl::PointCloud<descriptor_type>::Ptr& detection_descriptors,
    const modelify::PointSurfelCloudType::Ptr& object_keypoints,
    const typename pcl::PointCloud<descriptor_type>::Ptr& object_descriptors,
    double similarity_threshold, const modelify::CorrespondencesTypePtr& correspondences) {
  CHECK(detection_keypoints);
  CHECK(detection_descriptors);
  CHECK(object_keypoints);
  CHECK(object_descriptors);
  CHECK(correspondences);

  // Get correspondences
  *correspondences = computeCorrespondences<descriptor_type>(
      detection_keypoints, detection_descriptors, object_keypoints, object_descriptors,
      similarity_threshold);
  if (correspondences->empty()) {
    LOG(ERROR) << "No correspondences found!";
    return Transformation();
  }

  // Get correspondences based on RANSAC model
  std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
  modelify::registration_toolbox::RansacParams ransac_params;
  ransac_params.refine_model = false;
  ransac_params.max_correspondence_distance_m = 0.01;
  ransac_params.max_number_iterations = 1e3;
  modelify::CorrespondencesTypePtr ransac_correspondences(new modelify::CorrespondencesType());
  Eigen::Matrix4f transform_ransac;
  if (!modelify::registration_toolbox::filterCorrespondences(object_keypoints, detection_keypoints,
                                                             ransac_params, correspondences,
                                                             ransac_correspondences, &transform_ransac)) {
    LOG(WARNING) << "Failed to find a RANSAC model!";
    return Transformation();
  }

  *correspondences = *ransac_correspondences;
  LOG(INFO) << "Found RANSAC model based on " << correspondences->size() << " correspondences.";

  if (!Quaternion::isValidRotationMatrix(transform_ransac.block<3, 3>(0, 0))) {
    for (int i = 0; i < 3; ++i) {
      transform_ransac.block<3, 1>(0, i) =
          transform_ransac.block<3, 1>(0, i).normalized();
    }
    LOG(WARNING) << "Normalized rotation matrix, new transformation:\n"
                 << transform_ransac;
  }

  LOG(INFO) << "Time geometric consistency: "
            << std::chrono::duration<float>(std::chrono::steady_clock::now() - start).count()
            << " s";
  return Transformation(transform_ransac);
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
  CHECK(detection_surfels);
  CHECK(detection_keypoints);
  CHECK(detection_descriptors);
  CHECK(object_surfels);
  CHECK(object_keypoints);
  CHECK(object_descriptors);
  CHECK(correspondences);

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
              object_surfels, object_keypoints, detection_surfels, detection_keypoints,
              detection_descriptors, object_descriptors, fgr_params, &corrs, &transform)) {
    LOG(ERROR) << "Fast global registration was not successful!";
    return Transformation();
  }
  LOG(INFO) << "Time FGR: "
            << std::chrono::duration<float>(std::chrono::steady_clock::now() - start).count()
            << " s";

  for (const modelify::CorrespondencePair& correspondence : corrs) {
    pcl::Correspondence corr;
    corr.index_match = correspondence.first;
    corr.index_query = correspondence.second;
    correspondences->push_back(corr);
  }

  return Transformation(transform);
}

template <typename descriptor_type>
Transformation computeTransformUsingTeaser(
    const modelify::PointSurfelCloudType::Ptr& detection_keypoints,
    const typename pcl::PointCloud<descriptor_type>::Ptr& detection_descriptors,
    const modelify::PointSurfelCloudType::Ptr& object_keypoints,
    const typename pcl::PointCloud<descriptor_type>::Ptr& object_descriptors,
    double similarity_threshold, const modelify::CorrespondencesTypePtr& correspondences) {
  CHECK(detection_keypoints);
  CHECK(detection_descriptors);
  CHECK(object_keypoints);
  CHECK(object_descriptors);
  CHECK(correspondences);

  // Get correspondences
  *correspondences = computeCorrespondences<descriptor_type>(
      detection_keypoints, detection_descriptors, object_keypoints, object_descriptors,
      similarity_threshold);
  if (correspondences->empty()) {
    LOG(ERROR) << "No correspondences found!";
    return Transformation();
  }

  // Limit number of correspondences
  constexpr size_t max_correspondences = 5e3;
  if (correspondences->size() > max_correspondences) {
    if ((correspondences->size() - max_correspondences) < max_correspondences) {
      while (correspondences->size() > max_correspondences) {
        size_t idx = std::rand()/RAND_MAX;
        correspondences->erase(correspondences->begin() + idx);
      }
    } else {
      modelify::CorrespondencesType corr;
      for (size_t i = 0; i < max_correspondences; ++i) {
        size_t idx = std::rand()/RAND_MAX;
        corr.template emplace_back(correspondences->at(idx));
      }
      *correspondences = corr;
    }
    LOG(WARNING) << "Limited correspondences to " << correspondences->size() << " for Teaser.";
  }

  // Convert keypoints into matrix
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  Eigen::Matrix<double, 3, Eigen::Dynamic> object_matrix(3, correspondences->size());
  Eigen::Matrix<double, 3, Eigen::Dynamic> detection_matrix(3, correspondences->size());
  uint idx = 0;
  for (const auto& correspondence : *correspondences) {
    object_matrix.col(idx) << object_keypoints->points[correspondence.index_match].x,
        object_keypoints->points[correspondence.index_match].y,
        object_keypoints->points[correspondence.index_match].z;
    detection_matrix.col(idx) << detection_keypoints->points[correspondence.index_query].x,
        detection_keypoints->points[correspondence.index_query].y,
        detection_keypoints->points[correspondence.index_query].z;
    ++idx;
  }
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  LOG(INFO) << "Time conversion: " << std::chrono::duration<float>(end - begin).count() << " s";

  // Solve with TEASER++
  teaser::RobustRegistrationSolver::Params params;
  params.estimate_scaling = false;
  params.rotation_cost_threshold = 0.005;
  teaser::RobustRegistrationSolver solver(params);
  begin = std::chrono::steady_clock::now();
  teaser::RegistrationSolution solution = solver.solve(object_matrix, detection_matrix);
  end = std::chrono::steady_clock::now();
  LOG(INFO) << "Time teaser: " << std::chrono::duration<float>(end - begin).count() << " s";

  if (!solution.valid) {
    LOG(ERROR) << "Registration using Teaser failed!";
    return Transformation();
  }

  Eigen::Matrix3f rotation_matrix = solution.rotation.cast<float>();
  Transformation T_teaser =
      Transformation(solution.translation.cast<float>(), Quaternion(rotation_matrix));
  return T_teaser;
}

template <typename descriptor_type>
bool compute3dFeatures(const KeypointType& keypoint_type,
                       const modelify::PointSurfelCloudType::Ptr& pointcloud_surfel_ptr,
                       const modelify::PointSurfelCloudType::Ptr& keypoints,
                       const typename pcl::PointCloud<descriptor_type>::Ptr& descriptors) {
  CHECK(pointcloud_surfel_ptr);
  CHECK(keypoints);
  CHECK(descriptors);

  std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
  *keypoints = computeKeypoints(keypoint_type, pointcloud_surfel_ptr);
  LOG(INFO) << "Time keypoints: "
            << std::chrono::duration<float>(std::chrono::steady_clock::now() - start).count()
            << " s";

  std::chrono::steady_clock::time_point start_descriptors = std::chrono::steady_clock::now();
  *descriptors = computeDescriptors<descriptor_type>(pointcloud_surfel_ptr, keypoints);
  LOG(INFO)
      << "Time descriptors: "
      << std::chrono::duration<float>(std::chrono::steady_clock::now() - start_descriptors).count()
      << " s";
  LOG(INFO) << "Time 3D features: "
            << std::chrono::duration<float>(std::chrono::steady_clock::now() - start).count()
            << " s";
  return true;
}

}  // namespace cad_percept::object_detection
#endif  // CPT_OBJECT_DETECTION_OBJECT_DETECTION_INL_H_
