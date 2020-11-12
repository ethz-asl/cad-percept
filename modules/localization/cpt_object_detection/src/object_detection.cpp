#include "cpt_object_detection/object_detection.h"

#include <CGAL/linear_least_squares_fitting_3.h>
#include <cpt_utils/cpt_utils.h>
#include <modelify/feature_toolbox/descriptor_toolbox_3d.h>
#include <modelify/feature_toolbox/keypoint_toolbox_3d.h>
#include <modelify/registration_toolbox/registration_toolbox.h>
#include <pcl/common/pca.h>

namespace cad_percept {
namespace object_detection {

Transformation alignDetectionUsingPcaAndIcp(
    const cgal::MeshModel::Ptr& mesh_model,
    const pcl::PointCloud<pcl::PointXYZ>& detection_pointcloud, const std::string& config_file,
    Transformation* T_object_detection_init) {
  CHECK(T_object_detection_init);
  std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();

  // Get initial guess with PCA
  Transformation T_detection_object_pca = pca(mesh_model, detection_pointcloud);
  *T_object_detection_init = T_detection_object_pca.inverse();

  // Get final alignment with ICP
  Transformation T_object_detection =
      icp(mesh_model, detection_pointcloud, *T_object_detection_init, config_file);

  LOG(INFO) << "Time matching total: "
            << std::chrono::duration<float>(std::chrono::steady_clock::now() - time_start).count()
            << " s";
  return T_object_detection;
}

Transformation alignDetectionUsingPcaAndIcp(
    const cgal::MeshModel::Ptr& mesh_model,
    const pcl::PointCloud<pcl::PointXYZ>& detection_pointcloud) {
  Transformation T;
  std::string config_file;
  return alignDetectionUsingPcaAndIcp(mesh_model, detection_pointcloud, config_file, &T);
}

Transformation alignDetectionUsingPcaAndIcp(
    const cgal::MeshModel::Ptr& mesh_model,
    const pcl::PointCloud<pcl::PointXYZ>& detection_pointcloud, const std::string& config_file) {
  Transformation T;
  return alignDetectionUsingPcaAndIcp(mesh_model, detection_pointcloud, config_file, &T);
}

Transformation pca(const cgal::MeshModel::Ptr& mesh_model,
                   const pcl::PointCloud<pcl::PointXYZ>& detection_pointcloud) {
  CHECK_NOTNULL(mesh_model);
  std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();

  if (detection_pointcloud.size() < 3) {
    LOG(WARNING) << "Detection PCA not possible! Too few points: " << detection_pointcloud.size();
    return Transformation();
  }
  if (mesh_model->getMesh().empty()) {
    LOG(WARNING) << "Object PCA not possible! Mesh is empty.";
    return Transformation();
  }

  // Get data from detection pointcloud
  pcl::PCA<pcl::PointXYZ> pca_detection;
  pcl::PointCloud<pcl::PointXYZ>::Ptr detection_pointcloud_ptr = detection_pointcloud.makeShared();
  pca_detection.setInputCloud(detection_pointcloud_ptr);
  Eigen::Vector4f detection_centroid = pca_detection.getMean();
  Eigen::Matrix3f detection_vectors = pca_detection.getEigenVectors();

  // Get coordinate system to be right
  if (detection_vectors.determinant() < 0) {
    detection_vectors.col(2) = -detection_vectors.col(2);
  }

  // Get triangle vector from mesh
  std::vector<CGAL::Simple_cartesian<double>::Triangle_3> triangles;
  triangles.reserve(mesh_model->size());
  for (const auto& id : mesh_model->getFacetIds()) {
    triangles.push_back(mesh_model->getTriangle(id));
  }

  // Compute PCA for object triangles
  CGAL::Simple_cartesian<double>::Plane_3 plane;
  CGAL::Simple_cartesian<double>::Point_3 object_centroid;
  CGAL::linear_least_squares_fitting_3(triangles.begin(), triangles.end(), plane, object_centroid,
                                       CGAL::Dimension_tag<2>());

  // Get coordinate system from PCA
  Eigen::Matrix3f object_vectors;
  object_vectors.col(0) =
      Eigen::Vector3f(plane.base1().x(), plane.base1().y(), plane.base1().z()).normalized();
  object_vectors.col(1) =
      Eigen::Vector3f(plane.base2().x(), plane.base2().y(), plane.base2().z()).normalized();
  object_vectors.col(2) =
      Eigen::Vector3f(plane.orthogonal_vector().x(), plane.orthogonal_vector().y(),
                      plane.orthogonal_vector().z())
          .normalized();

  // Translation from mean of pointclouds det_r_obj_det
  kindr::minimal::PositionTemplate<float> translation(
      detection_centroid.head(3) -
      Eigen::Vector3f(object_centroid.x(), object_centroid.y(), object_centroid.z()));

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

  LOG(INFO) << "Time PCA: "
            << std::chrono::duration<float>(std::chrono::steady_clock::now() - time_start).count()
            << " s";
  return Transformation(rotation, translation);
}

Transformation icp(const cgal::MeshModel::Ptr& mesh_model,
                   const pcl::PointCloud<pcl::PointXYZ>& detection_pointcloud,
                   const Transformation& T_object_detection_init, const std::string& config_file) {
  CHECK(mesh_model);
  std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();

  // setup data points
  int n_points = detection_pointcloud.size();
  PM::DataPoints points_object = sampleDataPointsFromMesh(mesh_model, n_points);
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
  } catch (PM::ConvergenceError& error_msg) {
    LOG(ERROR) << "ICP was not successful!";
    return T_object_detection_init;
  }

  if (icp.getMaxNumIterationsReached()) {
    LOG(WARNING) << "ICP reached maximum number of iterations!";
  }

  if (!Quaternion::isValidRotationMatrix(T_object_detection_icp.block<3, 3>(0, 0))) {
    LOG(ERROR) << "Invalid rotation matrix!";
    return T_object_detection_init;
  }
  Transformation::TransformationMatrix Tmatrix(T_object_detection_icp);

  LOG(INFO) << "Time ICP: "
            << std::chrono::duration<float>(std::chrono::steady_clock::now() - time_start).count()
            << " s";
  return Transformation(Tmatrix);
}

Transformation icp(const pcl::PointCloud<pcl::PointXYZ>& object_pointcloud,
                   const pcl::PointCloud<pcl::PointXYZ>& detection_pointcloud,
                   const Transformation& T_object_detection_init) {
  std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(object_pointcloud));
  icp.setInputTarget(boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(detection_pointcloud));

  pcl::PointCloud<pcl::PointXYZ> pcl;
  icp.align(pcl, T_object_detection_init.getTransformationMatrix());

  if (!icp.hasConverged()) {
    LOG(WARNING) << "ICP hasn't converged!";
  }

  Eigen::Matrix4f T_object_detection_icp = icp.getFinalTransformation();

  if (!Quaternion::isValidRotationMatrix(T_object_detection_icp.block<3, 3>(0, 0))) {
    LOG(ERROR) << "Invalid rotation matrix!";
    return T_object_detection_init;
  }
  Transformation::TransformationMatrix Tmatrix(T_object_detection_icp);

  LOG(INFO) << "Time ICP: "
            << std::chrono::duration<float>(std::chrono::steady_clock::now() - time_start).count()
            << " s";
  return Transformation(Tmatrix);
}

PM::DataPoints sampleDataPointsFromMesh(const cgal::MeshModel::Ptr& mesh_model,
                                        const int number_of_points) {
  CHECK(mesh_model);

  // Sample points from mesh
  std::vector<cgal::Point> points;
  cpt_utils::samplePointsFromMesh(mesh_model->getMesh(), number_of_points, 0, &points);
  return convertMeshPointsToDataPoints(mesh_model, points);
}

PM::DataPoints convertMeshPointsToDataPoints(const cgal::MeshModel::Ptr& mesh_model,
                                             const std::vector<cgal::Point>& points) {
  CHECK(mesh_model);

  // Define features and descriptors
  PM::DataPoints::Labels feature_labels;
  feature_labels.push_back(PM::DataPoints::Label("x", 1));
  feature_labels.push_back(PM::DataPoints::Label("y", 1));
  feature_labels.push_back(PM::DataPoints::Label("z", 1));
  feature_labels.push_back(PM::DataPoints::Label("pad", 1));
  PM::DataPoints::Labels descriptor_labels;
  descriptor_labels.push_back(PM::DataPoints::Label("normals", 3));

  PM::Matrix features(feature_labels.totalDim(), points.size());
  PM::Matrix descriptors(descriptor_labels.totalDim(), points.size());
  for (int i = 0; i < points.size(); ++i) {
    features.col(i) = Eigen::Vector4f(points[i].x(), points[i].y(), points[i].z(), 1);
    cgal::PointAndPrimitiveId ppid = mesh_model->getClosestTriangle(points[i]);
    cgal::Vector normal = mesh_model->getNormal(ppid);
    descriptors.col(i) = Eigen::Vector3f(normal.x(), normal.y(), normal.z());
  }

  return PM::DataPoints(features, feature_labels, descriptors, descriptor_labels);
}

PM::DataPoints convertMeshToDataPoints(const cgal::MeshModel::Ptr& mesh_model) {
  CHECK(mesh_model);

  // Define feature and descriptor labels
  PM::DataPoints::Labels feature_labels;
  feature_labels.push_back(PM::DataPoints::Label("x", 1));
  feature_labels.push_back(PM::DataPoints::Label("y", 1));
  feature_labels.push_back(PM::DataPoints::Label("z", 1));
  feature_labels.push_back(PM::DataPoints::Label("pad", 1));

  PM::DataPoints::Labels descriptor_labels;
  descriptor_labels.push_back(PM::DataPoints::Label("normals", 3));

  // Get data from mesh
  PM::Matrix features(feature_labels.totalDim(), mesh_model->size());
  PM::Matrix descriptors(descriptor_labels.totalDim(), mesh_model->size());
  size_t i = 0;
  std::chrono::steady_clock::time_point conversion_start = std::chrono::steady_clock::now();
  for (const auto& id : mesh_model->getFacetIds()) {
    CGAL::Simple_cartesian<double>::Triangle_3 triangle = mesh_model->getTriangle(id);
    CGAL::Simple_cartesian<double>::Point_3 centroid = CGAL::centroid(triangle);
    CGAL::Simple_cartesian<double>::Vector_3 normal =
        triangle.supporting_plane().orthogonal_vector();

    features.col(i) = Eigen::Vector4f(centroid.x(), centroid.y(), centroid.z(), 1);
    descriptors.col(i) = Eigen::Vector3f(normal.x(), normal.y(), normal.z());
    ++i;
  }
  LOG(INFO)
      << "Time conversion mesh to pointmatcher: "
      << std::chrono::duration<float>(std::chrono::steady_clock::now() - conversion_start).count()
      << " s";

  return PM::DataPoints(features, feature_labels, descriptors, descriptor_labels);
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
  LOG(INFO)
      << "Time conversion PCL  to pointmatcher: "
      << std::chrono::duration<float>(std::chrono::steady_clock::now() - conversion_start).count()
      << " s";

  return PM::DataPoints(features, feature_labels);
}

Transformation refineUsingICP(const cgal::MeshModel::Ptr& mesh_model,
                              const modelify::PointSurfelCloudType::Ptr& detection_pointcloud,
                              const modelify::PointSurfelCloudType::Ptr& object_pointcloud,
                              const Transformation& transform_init,
                              const std::string& config_file) {
  // Validate initial alignment
  modelify::registration_toolbox::ICPParams icp_params;
  double cloud_resolution = modelify::kInvalidCloudResolution;
  double mean_squared_distance;
  double inlier_ratio;
  std::vector<size_t> outlier_indices;
  modelify::registration_toolbox::validateAlignment<modelify::PointSurfelType>(
      detection_pointcloud, object_pointcloud, transform_init.getTransformationMatrix(), icp_params,
      cloud_resolution, &mean_squared_distance, &inlier_ratio, &outlier_indices);
  LOG(INFO) << "Initial validation results: \n"
            << mean_squared_distance << " mean squared distance, " << inlier_ratio
            << " inlier ratio";

  // Refine transformation with ICP
  pcl::PointCloud<pcl::PointXYZ> detection_xyz;
  pcl::copyPointCloud(*detection_pointcloud, detection_xyz);
  Transformation transform_icp = icp(mesh_model, detection_xyz, transform_init, config_file);

  // Validate alignment ICP
  double mean_squared_distance_icp = 0;
  double inlier_ratio_icp = mean_squared_distance;
  modelify::registration_toolbox::validateAlignment<modelify::PointSurfelType>(
      detection_pointcloud, object_pointcloud, transform_icp.getTransformationMatrix(), icp_params,
      cloud_resolution, &mean_squared_distance_icp, &inlier_ratio_icp, &outlier_indices);
  LOG(INFO) << "ICP validation results: \n"
            << mean_squared_distance_icp << " mean squared distance, " << inlier_ratio_icp
            << " inlier ratio";

  if (inlier_ratio_icp >= inlier_ratio) {
    // TODO(gasserl): include mean_squared_distance_icp < mean_squared_distance
    return transform_icp;
  } else {
    LOG(WARNING) << "ICP didn't improve alignment!";
  }

  return transform_init;
}

modelify::PointSurfelCloudType estimateNormals(
    const pcl::PointCloud<pcl::PointXYZ>& pointcloud_xyz) {
  std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
  // Get surfels
  pcl::NormalEstimation<pcl::PointXYZ, modelify::PointSurfelType> normal_estimator;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr pcl_ptr =
      boost::make_shared<const pcl::PointCloud<pcl::PointXYZ>>(pointcloud_xyz);
  normal_estimator.setInputCloud(pcl_ptr);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZ>());
  normal_estimator.setSearchMethod(kd_tree);
  constexpr int k_radius = 4;
  normal_estimator.setKSearch(k_radius);

  modelify::PointSurfelCloudType pointcloud_surfels;
  normal_estimator.compute(pointcloud_surfels);
  for (size_t i = 0; i < pointcloud_xyz.size(); ++i) {
    pointcloud_surfels.points[i].x = pointcloud_xyz.points[i].x;
    pointcloud_surfels.points[i].y = pointcloud_xyz.points[i].y;
    pointcloud_surfels.points[i].z = pointcloud_xyz.points[i].z;
  }

  size_t size_before = pointcloud_surfels.size();
  size_t i = 0;
  while (i < pointcloud_surfels.size()) {
    if (std::isnan(pointcloud_surfels.points[i].normal_x) ||
        std::isnan(pointcloud_surfels.points[i].normal_y) ||
        std::isnan(pointcloud_surfels.points[i].normal_z) ||
        std::isnan(pointcloud_surfels.points[i].x) || std::isnan(pointcloud_surfels.points[i].y) ||
        std::isnan(pointcloud_surfels.points[i].z)) {
      pointcloud_surfels.points.erase(pointcloud_surfels.points.begin() + i);
    } else {
      ++i;
    }
  }
  if (size_before > pointcloud_surfels.size()) {
    LOG(INFO) << "Filtered " << size_before - pointcloud_surfels.size()
              << " points with NaN points or normals";
  }
  LOG(INFO) << "Time normals: "
            << std::chrono::duration<float>(std::chrono::steady_clock::now() - start).count()
            << " s";
  return pointcloud_surfels;
}

modelify::PointSurfelCloudType getKeypoints(
    const KeypointType& keypoint_type,
    const modelify::PointSurfelCloudType::Ptr& pointcloud_surfel_ptr) {
  modelify::PointSurfelCloudType::Ptr keypoints(new modelify::PointSurfelCloudType());
  switch (keypoint_type) {
    case kIss: {
      modelify::feature_toolbox::IssParams iss_params;
      if (!modelify::feature_toolbox::detectKeypointsISS(pointcloud_surfel_ptr, iss_params,
                                                         keypoints)) {
        LOG(WARNING) << "Could not extract ISS keypoints!";
      }
      break;
    }
    case kHarris: {
      modelify::feature_toolbox::Harris3dParams harris_params;
      if (!modelify::feature_toolbox::extractHarrisKeypoints(harris_params, pointcloud_surfel_ptr,
                                                             keypoints)) {
        LOG(WARNING) << "Could not extract Harris keypoints!";
      }
      break;
    }
    case kUniform: {
      modelify::feature_toolbox::UniformDownsamplingParams uniform_params;
      if (!modelify::feature_toolbox::getKeypointsFromUniformDownsampling(
              pointcloud_surfel_ptr, uniform_params, keypoints)) {
        LOG(WARNING) << "Could not extract uniform keypoints!";
      }
      break;
    }
    default:
      LOG(ERROR) << "Unknown keypoint type! " << keypoint_type;
      return *keypoints;
  }
  LOG(INFO) << "Extracted " << keypoints->size() << " keypoints";
  return *keypoints;
}

template <>
pcl::PointCloud<modelify::DescriptorSHOT> getDescriptors<modelify::DescriptorSHOT>(
    const modelify::PointSurfelCloudType::Ptr& pointcloud_surfel_ptr,
    const modelify::PointSurfelCloudType::Ptr& keypoints) {
  modelify::feature_toolbox::SHOTParams shot_params;
  const pcl::PointCloud<modelify::DescriptorSHOT>::Ptr descriptors(
      new pcl::PointCloud<modelify::DescriptorSHOT>());
  modelify::feature_toolbox::describeKeypoints<modelify::DescriptorSHOT>(
      pointcloud_surfel_ptr, modelify::kInvalidCloudResolution, shot_params, keypoints,
      descriptors);
  return *descriptors;
}

template <>
pcl::PointCloud<modelify::DescriptorFPFH> getDescriptors<modelify::DescriptorFPFH>(
    const modelify::PointSurfelCloudType::Ptr& pointcloud_surfel_ptr,
    const modelify::PointSurfelCloudType::Ptr& keypoints) {
  modelify::feature_toolbox::FPFHParams fpfh_params;
  const pcl::PointCloud<modelify::DescriptorFPFH>::Ptr descriptors(
      new pcl::PointCloud<modelify::DescriptorFPFH>());
  modelify::feature_toolbox::describeKeypoints<modelify::DescriptorFPFH>(
      pointcloud_surfel_ptr, modelify::kInvalidCloudResolution, fpfh_params, keypoints,
      descriptors);
  return *descriptors;
}

}  // namespace object_detection
}  // namespace cad_percept