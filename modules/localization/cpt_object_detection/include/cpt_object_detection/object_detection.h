#ifndef CPT_OBJECT_DETECTION_MODULES_LOCALIZATION_CPT_OBJECT_DETECTION_INCLUDE_CPT_OBJECT_DETECTION_OBJECT_DETECTION_H_
#define CPT_OBJECT_DETECTION_MODULES_LOCALIZATION_CPT_OBJECT_DETECTION_INCLUDE_CPT_OBJECT_DETECTION_OBJECT_DETECTION_H_

#include <cgal_definitions/mesh_model.h>
#include <kindr/minimal/quat-transformation.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pointmatcher/PointMatcher.h>

namespace cad_percept {
namespace object_detection {

typedef kindr::minimal::QuatTransformationTemplate<float> Transformation;
typedef kindr::minimal::RotationQuaternionTemplate<float> Quaternion;
typedef PointMatcher<float> PM;

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

PM::DataPoints sampleDataPointsFromMesh(const cgal::MeshModel::Ptr& mesh_model,
                                        const int number_of_points);
PM::DataPoints convertMeshToDataPoints(const cgal::MeshModel::Ptr& mesh_model);
PM::DataPoints convertMeshPointsToDataPoints(const cgal::MeshModel::Ptr& mesh_model,
                                             const std::vector<cgal::Point>& points);
PM::DataPoints convertPclToDataPoints(const pcl::PointCloud<pcl::PointXYZ>& pointcloud);

}  // namespace object_detection
}  // namespace cad_percept
#endif  // CPT_OBJECT_DETECTION_MODULES_LOCALIZATION_CPT_OBJECT_DETECTION_INCLUDE_CPT_OBJECT_DETECTION_OBJECT_DETECTION_H_
