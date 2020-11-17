#ifndef CPT_OBJECT_DETECTION_OBJECT_DETECTION_H_
#define CPT_OBJECT_DETECTION_OBJECT_DETECTION_H_

#include <cgal_definitions/mesh_model.h>
#include <kindr/minimal/quat-transformation.h>
#include <modelify/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pointmatcher/PointMatcher.h>

namespace cad_percept::object_detection {

typedef kindr::minimal::QuatTransformationTemplate<float> Transformation;
typedef kindr::minimal::RotationQuaternionTemplate<float> Quaternion;
typedef PointMatcher<float> PM;

enum KeypointType { kIss = 0, kHarris, kUniform, kNumKeypointTypes };
enum DescriptorType { kFpfh = 0, kShot, k3dSmoothNet, kNumDescriptorTypes };
enum MatchingMethod {
  kGeometricConsistency = 0,
  kFastGlobalRegistration,
  kTeaser,
  kNumMatchingMethods
};

Transformation alignDetectionUsingPcaAndIcp(
    const cgal::MeshModel::Ptr& mesh_model,
    const pcl::PointCloud<pcl::PointXYZ>& detection_pointcloud);
Transformation alignDetectionUsingPcaAndIcp(
    const cgal::MeshModel::Ptr& mesh_model,
    const pcl::PointCloud<pcl::PointXYZ>& detection_pointcloud, const std::string& config_file);
Transformation alignDetectionUsingPcaAndIcp(
    const cgal::MeshModel::Ptr& mesh_model,
    const pcl::PointCloud<pcl::PointXYZ>& detection_pointcloud, const std::string& config_file,
    Transformation* T_object_detection_init);

Transformation pca(const cgal::MeshModel::Ptr& mesh_model,
                   const pcl::PointCloud<pcl::PointXYZ>& detection_pointcloud);
Transformation icp(const cgal::MeshModel::Ptr& mesh_model,
                   const pcl::PointCloud<pcl::PointXYZ>& detection_pointcloud,
                   const Transformation& T_object_detection_init, const std::string& config_file);
Transformation icp(const pcl::PointCloud<pcl::PointXYZ>& object_pointcloud,
                   const pcl::PointCloud<pcl::PointXYZ>& detection_pointcloud,
                   const Transformation& T_object_detection_init);

PM::DataPoints sampleDataPointsFromMesh(const cgal::MeshModel::Ptr& mesh_model,
                                        const int number_of_points);
PM::DataPoints convertMeshToDataPoints(const cgal::MeshModel::Ptr& mesh_model);
PM::DataPoints convertMeshPointsToDataPoints(const cgal::MeshModel::Ptr& mesh_model,
                                             const std::vector<cgal::Point>& points);
PM::DataPoints convertPclToDataPoints(const pcl::PointCloud<pcl::PointXYZ>& pointcloud);

template <typename descriptor_type>
Transformation computeTransformUsing3dFeatures(
    MatchingMethod matching_method, const modelify::PointSurfelCloudType::Ptr& detection_surfels,
    const modelify::PointSurfelCloudType::Ptr& detection_keypoints,
    const typename pcl::PointCloud<descriptor_type>::Ptr& detection_descriptors,
    const modelify::PointSurfelCloudType::Ptr& object_surfels,
    const modelify::PointSurfelCloudType::Ptr& object_keypoints,
    const typename pcl::PointCloud<descriptor_type>::Ptr& object_descriptors,
    double similarity_threshold, const modelify::CorrespondencesTypePtr& correspondences);
template <typename descriptor_type>
modelify::CorrespondencesType computeCorrespondences(
    const modelify::PointSurfelCloudType::Ptr& detection_keypoints,
    const typename pcl::PointCloud<descriptor_type>::Ptr& detection_descriptors,
    const modelify::PointSurfelCloudType::Ptr& object_keypoints,
    const typename pcl::PointCloud<descriptor_type>::Ptr& object_descriptors,
    double similarity_threshold);
template <typename descriptor_type>
Transformation computeTransformUsingFgr(
    const modelify::PointSurfelCloudType::Ptr& detection_surfels,
    const modelify::PointSurfelCloudType::Ptr& detection_keypoints,
    const typename pcl::PointCloud<descriptor_type>::Ptr& detection_descriptors,
    const modelify::PointSurfelCloudType::Ptr& object_surfels,
    const modelify::PointSurfelCloudType::Ptr& object_keypoints,
    const typename pcl::PointCloud<descriptor_type>::Ptr& object_descriptors,
    const modelify::CorrespondencesTypePtr& correspondences);
template <typename descriptor_type>
Transformation computeTransformUsingGeometricConsistency(
    const modelify::PointSurfelCloudType::Ptr& detection_keypoints,
    const typename pcl::PointCloud<descriptor_type>::Ptr& detection_descriptors,
    const modelify::PointSurfelCloudType::Ptr& object_keypoints,
    const typename pcl::PointCloud<descriptor_type>::Ptr& object_descriptors,
    double similarity_threshold, const modelify::CorrespondencesTypePtr& correspondences);
template <typename descriptor_type>
Transformation computeTransformUsingTeaser(
    const modelify::PointSurfelCloudType::Ptr& detection_keypoints,
    const typename pcl::PointCloud<descriptor_type>::Ptr& detection_descriptors,
    const modelify::PointSurfelCloudType::Ptr& object_keypoints,
    const typename pcl::PointCloud<descriptor_type>::Ptr& object_descriptors,
    double similarity_threshold, const modelify::CorrespondencesTypePtr& correspondences);
Transformation refineUsingICP(const cgal::MeshModel::Ptr& mesh_model,
                              const modelify::PointSurfelCloudType::Ptr& detection_surfels,
                              const modelify::PointSurfelCloudType::Ptr& object_surfels,
                              const Transformation& transform_init, const std::string& config_file);

template <typename descriptor_type>
bool get3dFeatures(const KeypointType& keypoint_type,
                   const modelify::PointSurfelCloudType::Ptr& pointcloud_surfel_ptr,
                   const modelify::PointSurfelCloudType::Ptr& keypoints,
                   const typename pcl::PointCloud<descriptor_type>::Ptr& descriptors);
modelify::PointSurfelCloudType estimateNormals(
    const pcl::PointCloud<pcl::PointXYZ>& pointcloud_xyz);
modelify::PointSurfelCloudType getKeypoints(
    const KeypointType& keypoint_type,
    const modelify::PointSurfelCloudType::Ptr& pointcloud_surfel_ptr);
template <typename descriptor_type>
typename pcl::PointCloud<descriptor_type> getDescriptors(
    const modelify::PointSurfelCloudType::Ptr& pointcloud_surfel_ptr,
    const modelify::PointSurfelCloudType::Ptr& keypoints);

}  // namespace cad_percept::object_detection

#include "cpt_object_detection/object_detection_inl.h"

#endif  // CPT_OBJECT_DETECTION_OBJECT_DETECTION_H_
