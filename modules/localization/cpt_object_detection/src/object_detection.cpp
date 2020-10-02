#include "cpt_object_detection/object_detection.h"

#include <pcl/common/pca.h>
#include <modelify/feature_toolbox/keypoint_toolbox_3d.h>
#include <modelify/feature_toolbox/descriptor_toolbox_3d.h>
#include <modelify/registration_toolbox/registration_toolbox.h>

namespace cad_percept {
namespace object_detection {

Transformation alignDetectionUsingPcaAndIcp(
    const pcl::PointCloud<pcl::PointXYZ>& object_pointcloud,
    const pcl::PointCloud<pcl::PointXYZ>& detection_pointcloud, const std::string& config_file,
    Transformation* T_object_detection_init) {
  CHECK(T_object_detection_init);
  std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();

  // Get initial guess with PCA
  Transformation T_detection_object_pca = pca(object_pointcloud, detection_pointcloud);
  *T_object_detection_init = T_detection_object_pca.inverse();

  // Get final alignment with ICP
  Transformation T_object_detection =
      icp(object_pointcloud, detection_pointcloud, *T_object_detection_init, config_file);

  LOG(INFO) << "Total matching time: " << (std::chrono::steady_clock::now() - time_start).count()
            << " s";
  return T_object_detection;
}

Transformation alignDetectionUsingPcaAndIcp(
    const pcl::PointCloud<pcl::PointXYZ>& object_pointcloud,
    const pcl::PointCloud<pcl::PointXYZ>& detection_pointcloud) {
  Transformation T;
  std::string config_file;
  return alignDetectionUsingPcaAndIcp(object_pointcloud, detection_pointcloud, config_file, &T);
}

Transformation alignDetectionUsingPcaAndIcp(
    const pcl::PointCloud<pcl::PointXYZ>& object_pointcloud,
    const pcl::PointCloud<pcl::PointXYZ>& detection_pointcloud, const std::string& config_file) {
  Transformation T;
  return alignDetectionUsingPcaAndIcp(object_pointcloud, detection_pointcloud, config_file, &T);
}

Transformation pca(const pcl::PointCloud<pcl::PointXYZ>& object_pointcloud,
                   const pcl::PointCloud<pcl::PointXYZ>& detection_pointcloud) {
  std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();

  if (detection_pointcloud.size() < 3) {
    LOG(WARNING) << "Detection PCA not possible! Too few points: " << detection_pointcloud.size();
    return Transformation();
  }
  if (object_pointcloud.size() < 3) {
    LOG(WARNING) << "Object PCA not possible! Too few points: " << object_pointcloud.size();
    return Transformation();
  }

  // Get data from detection pointcloud
  pcl::PCA<pcl::PointXYZ> pca_detection;
  pcl::PointCloud<pcl::PointXYZ>::Ptr detection_pointcloud_ptr = detection_pointcloud.makeShared();
  pca_detection.setInputCloud(detection_pointcloud_ptr);
  Eigen::Vector4f detection_centroid = pca_detection.getMean();
  Eigen::Matrix3f detection_vectors = pca_detection.getEigenVectors();

  // Get data from object pointcloud
  pcl::PCA<pcl::PointXYZ> pca_object;
  pcl::PointCloud<pcl::PointXYZ>::Ptr object_pointcloud_ptr = object_pointcloud.makeShared();
  pca_object.setInputCloud(object_pointcloud_ptr);
  Eigen::Vector4f object_centroid = pca_object.getMean();
  Eigen::Matrix3f object_vectors = pca_object.getEigenVectors();

  // Translation from mean of pointclouds det_r_obj_det
  kindr::minimal::PositionTemplate<float> translation(detection_centroid.head(3) -
                                                      object_centroid.head(3));

  // Get coordinate system to be right
  if (detection_vectors.determinant() < 0) {
    detection_vectors.col(2) = -detection_vectors.col(2);
  }
  if (object_vectors.determinant() < 0) {
    object_vectors.col(2) = -object_vectors.col(2);
  }
  // Get rotation between detection and object pointcloud
  Eigen::Matrix3f rotation_matrix = detection_vectors * object_vectors.transpose();

  Quaternion rotation;
  rotation.setIdentity();
  if (Quaternion::isValidRotationMatrix(rotation_matrix)) {
    rotation = Quaternion(rotation_matrix);
  } else {
    LOG(WARNING) << "Rotation matrix is not valid!";
    LOG(INFO) << "determinant: " << rotation_matrix.determinant();
    LOG(INFO) << "R*R^T - I:\n"
              << rotation_matrix * rotation_matrix.transpose() - Eigen::Matrix3f::Identity();
    return Transformation();
  }

  LOG(INFO) << "Time PCA: " << (std::chrono::steady_clock::now() - time_start).count() << " s";
  return Transformation(rotation, translation);
}

Transformation icp(const pcl::PointCloud<pcl::PointXYZ>& object_pointcloud,
                   const pcl::PointCloud<pcl::PointXYZ>& detection_pointcloud,
                   const Transformation& T_object_detection_init, const std::string& config_file) {
  std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();

  // setup data points
  PM::DataPoints points_object = convertPclToDataPoints(object_pointcloud);
  PM::DataPoints points_detection = convertPclToDataPoints(detection_pointcloud);

  // setup icp
  PM::ICP icp;
  if (!config_file.empty()) {
    std::ifstream ifs(config_file.c_str());
    if (ifs.good()) {
      icp.loadFromYaml(ifs);
    } else {
      LOG(ERROR) << "Cannot load ICP config from YAML file " << config_file;
      icp.setDefault();
    }
  } else {
    LOG(INFO) << "No ICP config file given, using default ICP settings";
    icp.setDefault();
  }

  // icp: reference - object mesh, data - detection cloud
  PM::TransformationParameters T_object_detection_icp;
  try {
    T_object_detection_icp = icp.compute(points_detection, points_object,
                                         T_object_detection_init.getTransformationMatrix());
    if (icp.getMaxNumIterationsReached()) {
      LOG(ERROR) << "ICP reached maximum number of iterations!";
    }
  } catch (PM::ConvergenceError& error_msg) {
    LOG(WARNING) << "ICP was not successful!";
    return T_object_detection_init;
  }

  if (!Quaternion::isValidRotationMatrix(T_object_detection_icp.block<3, 3>(0, 0))) {
    LOG(ERROR) << "Invalid rotation matrix!";
    return T_object_detection_init;
  }
  Transformation::TransformationMatrix Tmatrix(T_object_detection_icp);

  LOG(INFO) << "Time ICP: " << (std::chrono::steady_clock::now() - time_start).count() << " s";
  LOG(INFO) << "ICP on detection pointcloud and object mesh vertices successful!";
  return Transformation(Tmatrix);
}

PM::DataPoints convertPclToDataPoints(const pcl::PointCloud<pcl::PointXYZ>& pointcloud) {
  // Define feature labels
  PM::DataPoints::Labels feature_labels;
  feature_labels.push_back(PM::DataPoints::Label("x", 1));
  feature_labels.push_back(PM::DataPoints::Label("y", 1));
  feature_labels.push_back(PM::DataPoints::Label("z", 1));
  feature_labels.push_back(PM::DataPoints::Label("pad", 1));

  // Get features from pointcloud
  PM::Matrix features(feature_labels.totalDim(), pointcloud.size());
  std::chrono::steady_clock::time_point conversion_start = std::chrono::steady_clock::now();
  for (uint i = 0; i < pointcloud.size(); ++i) {
    features.col(i) =
        Eigen::Vector4f(pointcloud.points[i].x, pointcloud.points[i].y, pointcloud.points[i].z, 1);
  }
  LOG(INFO) << "Time conversion PCL  to pointmatcher: "
            << (std::chrono::steady_clock::now() - conversion_start).count() << " s";

  return PM::DataPoints(features, feature_labels);
}

Transformation computeTransformFromCorrespondences(
    const modelify::PointSurfelCloudType::Ptr& detection_keypoints,
    const modelify::PointSurfelCloudType::Ptr& object_keypoints,
    const modelify::CorrespondencesTypePtr& correspondences) {
  // Filter correspondences
  modelify::registration_toolbox::RansacParams ransac_params;
  modelify::CorrespondencesTypePtr filtered_correspondences(new modelify::CorrespondencesType());
  if (!modelify::registration_toolbox::filterCorrespondences(detection_keypoints, object_keypoints,
                                                             ransac_params, correspondences,
                                                             filtered_correspondences)) {
    // TODO(gasserl): error instead of warning?
    LOG(WARNING) << "Filtering correspondences failed!";
    *filtered_correspondences = *correspondences;
  }
  LOG(INFO) << "Filtered correspondences to " << filtered_correspondences->size();

  // Align features
  modelify::registration_toolbox::GeometricConsistencyParams consistency_params;
  consistency_params.consensus_set_resolution_m = 0.05;
  modelify::TransformationVector T_geometric_consistency;
  std::vector<modelify::CorrespondencesType> clustered_correspondences;
  if (!modelify::registration_toolbox::alignKeypointsGeometricConsistency(
          detection_keypoints, object_keypoints, filtered_correspondences,
          consistency_params.min_cluster_size, consistency_params.consensus_set_resolution_m,
          &T_geometric_consistency, &clustered_correspondences)) {
    LOG(ERROR) << "Keypoint alignment failed!";
    return Transformation();
  }
  *correspondences = clustered_correspondences[0];
  LOG(INFO) << "Aligned keypoints, best alignment found with "
            << clustered_correspondences[0].size() << " correspondences and transform\n"
            << T_geometric_consistency[0];

  if (!Quaternion::isValidRotationMatrix(T_geometric_consistency[0].block<3, 3>(0, 0))) {
    for (int i = 0; i < 3; ++i) {
      T_geometric_consistency[0].block<3, 1>(0, i) =
          T_geometric_consistency[0].block<3, 1>(0, i).normalized();
    }
    LOG(WARNING) << "Normalized rotation matrix, new transformation:\n"
                 << T_geometric_consistency[0];
  }
  return Transformation(T_geometric_consistency[0]);
}

Transformation icpUsingModelify(const modelify::PointSurfelCloudType::Ptr& detection_surfels,
                                const modelify::PointSurfelCloudType::Ptr& object_surfels,
                                const Transformation& transform_init) {
  // Validate initial alignment
  modelify::registration_toolbox::ICPParams icp_params;
  double cloud_resolution = modelify::kInvalidCloudResolution;
  double mean_squared_distance;
  double inlier_ratio;
  std::vector<size_t> outlier_indices;
  modelify::registration_toolbox::validateAlignment<modelify::PointSurfelType>(
      detection_surfels, object_surfels, transform_init.getTransformationMatrix(), icp_params,
      cloud_resolution, &mean_squared_distance, &inlier_ratio, &outlier_indices);
  LOG(INFO) << "Initial validation results: " << mean_squared_distance << " mean squared distance, "
            << inlier_ratio << " inlier ratio";

  // Refine transformation with modelify ICP
  modelify::Transformation T_icp;
  double mean_squared_distance_icp = 0;
  double inlier_ratio_icp = mean_squared_distance;
  if (!estimateTransformationPointToPoint(detection_surfels, object_surfels,
                                          transform_init.getTransformationMatrix(), icp_params,
                                          cloud_resolution, &T_icp, &mean_squared_distance_icp)) {
    LOG(WARNING) << "Keypoint ICP refinement failed!";
  } else {
    if (!Quaternion::isValidRotationMatrix(T_icp.block<3, 3>(0, 0))) {
      for (int i = 0; i < 3; ++i) {
        T_icp.block<3, 1>(0, i) = T_icp.block<3, 1>(0, i).normalized();
      }
    }
    Transformation transform_icp = Transformation(T_icp) * transform_init;

    // Validate alignment ICP
    modelify::registration_toolbox::validateAlignment<modelify::PointSurfelType>(
        detection_surfels, object_surfels, transform_icp.getTransformationMatrix(), icp_params,
        cloud_resolution, &mean_squared_distance_icp, &inlier_ratio_icp, &outlier_indices);
    LOG(INFO) << "ICP validation results: " << mean_squared_distance_icp
              << " mean squared distance, " << inlier_ratio_icp << " inlier ratio";

    if (inlier_ratio_icp >= inlier_ratio) {
      // && mean_squared_distance_icp < mean_squared_distance) {
      return transform_icp;
    } else {
      LOG(WARNING) << "ICP didn't improve alignment!";
    }
  }
  return transform_init;
}

bool getKeypoints(const KeypointType& keypoint_type,
                  const modelify::PointSurfelCloudType::Ptr& pointcloud_surfel_ptr,
                  const modelify::PointSurfelCloudType::Ptr& keypoints) {
  switch (keypoint_type) {
    case kIss:
      return getIssKeypoints(pointcloud_surfel_ptr, keypoints);
    case kHarris:
      return getHarrisKeypoints(pointcloud_surfel_ptr, keypoints);
    case kUniform:
      return getUniformKeypoints(pointcloud_surfel_ptr, keypoints);
    default:
      LOG(ERROR) << "Unknown keypoint type! " << keypoint_type;
      return false;
  }
}

bool getIssKeypoints(const modelify::PointSurfelCloudType::Ptr& pointcloud_surfel_ptr,
                     const modelify::PointSurfelCloudType::Ptr& keypoints) {
  modelify::feature_toolbox::IssParams iss_params;
  if (!modelify::feature_toolbox::detectKeypointsISS(pointcloud_surfel_ptr, iss_params,
                                                     keypoints)) {
    LOG(WARNING) << "Could not extract ISS keypoints!";
    return false;
  }
  LOG(INFO) << "Extracted " << keypoints->size() << " ISS keypoints";
  return true;
}

bool getHarrisKeypoints(const modelify::PointSurfelCloudType::Ptr& pointcloud_surfel_ptr,
                        const modelify::PointSurfelCloudType::Ptr& keypoints) {
  // Get keypoints
  modelify::feature_toolbox::Harris3dParams harris_params;
  if (!modelify::feature_toolbox::extractHarrisKeypoints(harris_params, pointcloud_surfel_ptr,
                                                         keypoints)) {
    LOG(WARNING) << "Could not extract Harris keypoints!";
    return false;
  }
  LOG(INFO) << "Extracted " << keypoints->size() << " Harris keypoints";

  /*LOG(INFO) << "Harris params:" << harris_params.toString();
  size_t num = 0;
  float curvature = 0;
  float min_curvature = 1e9;
  float max_curvature = 0;
  for (const auto& point : *pointcloud_surfel_ptr) {
    ++num;
    curvature += point.curvature;
    min_curvature = std::min(min_curvature, point.curvature);
    max_curvature = std::max(max_curvature, point.curvature);
  }
  LOG(INFO) << "Avg curvature: " << curvature / float(num);
  LOG(INFO) << "Min curvature: " << min_curvature;
  LOG(INFO) << "Max curvature: " << max_curvature;*/

  return true;
}

bool getUniformKeypoints(const modelify::PointSurfelCloudType::Ptr& pointcloud_surfel_ptr,
                         const modelify::PointSurfelCloudType::Ptr& keypoints) {
  // Get keypoints
  modelify::feature_toolbox::UniformDownsamplingParams uniform_params;
  if (!modelify::feature_toolbox::getKeypointsFromUniformDownsampling(pointcloud_surfel_ptr,
                                                                      uniform_params, keypoints)) {
    LOG(WARNING) << "Could not extract uniform keypoints!";
    return false;
  }
  LOG(INFO) << "Extracted " << keypoints->size() << " uniform keypoints";
  return true;
}

template <>
void getDescriptors<modelify::DescriptorSHOT>(
    const modelify::PointSurfelCloudType::Ptr& pointcloud_surfel_ptr,
    const modelify::PointSurfelCloudType::Ptr& keypoints,
    const pcl::PointCloud<modelify::DescriptorSHOT>::Ptr& descriptors) {
  modelify::feature_toolbox::SHOTParams shot_params;
  modelify::feature_toolbox::describeKeypoints<modelify::DescriptorSHOT>(
      pointcloud_surfel_ptr, modelify::kInvalidCloudResolution, shot_params, keypoints,
      descriptors);
}

template <>
void getDescriptors<modelify::DescriptorFPFH>(
    const modelify::PointSurfelCloudType::Ptr& pointcloud_surfel_ptr,
    const modelify::PointSurfelCloudType::Ptr& keypoints,
    const pcl::PointCloud<modelify::DescriptorFPFH>::Ptr& descriptors) {
  modelify::feature_toolbox::FPFHParams fpfh_params;
  modelify::feature_toolbox::describeKeypoints<modelify::DescriptorFPFH>(
      pointcloud_surfel_ptr, modelify::kInvalidCloudResolution, fpfh_params, keypoints,
      descriptors);
}

}  // namespace object_detection
}  // namespace cad_percept