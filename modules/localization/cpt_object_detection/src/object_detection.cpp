#include <pcl/common/pca.h>

#include "cpt_object_detection/object_detection.h"

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

}  // namespace object_detection
}  // namespace cad_percept