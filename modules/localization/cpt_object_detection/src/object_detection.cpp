#include "cpt_object_detection/object_detection.h"

#include <CGAL/linear_least_squares_fitting_3.h>
#include <cpt_object_detection/learned_descriptor.h>
#include <cpt_object_detection/unit_descriptor.h>
#include <cpt_utils/cpt_utils.h>
#include <modelify/feature_toolbox/descriptor_toolbox_3d.h>
#include <modelify/feature_toolbox/keypoint_toolbox_3d.h>
#include <modelify/registration_toolbox/registration_toolbox.h>
#include <pcl/common/pca.h>

#include <thread>

namespace cad_percept::object_detection {

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
  CGAL::Simple_cartesian<double>::Point_3 object_centroid_cgal;
  CGAL::linear_least_squares_fitting_3(triangles.begin(), triangles.end(), plane,
                                       object_centroid_cgal, CGAL::Dimension_tag<2>());
  Eigen::Vector3f object_centroid(object_centroid_cgal.x(), object_centroid_cgal.y(),
                                  object_centroid_cgal.z());

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

  // Get rotation between detection and object pointcloud
  Eigen::Matrix3f rotation_matrix = detection_vectors * object_vectors.transpose();

  // Translation from mean of pointclouds det_r_obj_det
  object_centroid = rotation_matrix * object_centroid;
  kindr::minimal::PositionTemplate<float> translation(detection_centroid.head(3) - object_centroid);

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

Transformation optimizeTransformation(const modelify::PointSurfelCloudType::Ptr& object_surfels,
                                      const modelify::PointSurfelCloudType::Ptr& detection_surfels,
                                      const Transformation& T_init) {
  std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();

  pcl::CentroidPoint<pcl::PointSurfel> centroid_cal;
  for (const auto& point : object_surfels->points) {
    centroid_cal.add(point);
  }
  pcl::PointSurfel centroid_pcl;
  centroid_cal.get(centroid_pcl);
  Position centroid(centroid_pcl.x, centroid_pcl.y, centroid_pcl.z);

  std::vector<Quaternion> rotations;
  rotations.emplace_back();
  rotations.emplace_back(0, 1, 0, 0);
  rotations.emplace_back(0, 0, 1, 0);
  rotations.emplace_back(0, 0, 0, 1);

  std::vector<Position> translations;
  translations.emplace_back();
  translations.emplace_back(0, 2 * centroid.y(), 2 * centroid.z());
  translations.emplace_back(2 * centroid.x(), 0, 2 * centroid.z());
  translations.emplace_back(2 * centroid.x(), 2 * centroid.y(), 0);

  std::vector<Transformation> transformations;
  transformations.reserve(4);
  std::vector<double> inlier_ratios;
  inlier_ratios.reserve(4);
  std::vector<double> mean_squared_distances;
  mean_squared_distances.reserve(4);
  std::vector<double> optimization_criteria;
  optimization_criteria.reserve(4);

  for (size_t i = 0; i < rotations.size(); ++i) {
    transformations[i] = Transformation(rotations[i], translations[i]);
    Transformation T_test = T_init * transformations[i];
    modelify::registration_toolbox::ICPParams icp_params;
    icp_params.inlier_distance_threshold_m = 0.01;
    double cloud_resolution = modelify::kInvalidCloudResolution;
    double mean_squared_distance;
    double inlier_ratio;
    std::vector<size_t> outlier_indices;
    modelify::registration_toolbox::validateAlignment<modelify::PointSurfelType>(
        object_surfels, detection_surfels, T_test.getTransformationMatrix(), icp_params,
        cloud_resolution, &mean_squared_distance, &inlier_ratio, &outlier_indices);
    // Save results
    inlier_ratios.emplace_back(inlier_ratio);
    mean_squared_distances.emplace_back(mean_squared_distance);
    optimization_criteria.emplace_back(inlier_ratio / mean_squared_distance);
    LOG(INFO) << "Inliers v" << i << ": " << inlier_ratio;
  }
  LOG(INFO) << "Time optimization: "
            << std::chrono::duration<float>(std::chrono::steady_clock::now() - time_start).count()
            << " s";

  // Select best orientation
  const auto& most_inliers = std::max_element(inlier_ratios.begin(), inlier_ratios.end());
  // TODO(gasserl): consider other criteria
  return T_init * transformations[std::distance(inlier_ratios.begin(), most_inliers)];
}

double computeInlierRatio(const bool use_pointcloud,
                          const modelify::PointSurfelCloudType::Ptr& object_surfels,
                          const modelify::PointSurfelCloudType::Ptr& detection_surfels,
                          const cgal::MeshModel& mesh_model,
                          const pcl::PointCloud<pcl::PointXYZ>& detection_pointcloud,
                          const Transformation& T_object_detection) {
  double inlier_ratio = 0;
  constexpr double inlier_dist = 0.01;

  if (use_pointcloud) {
    // Compute inliers
    modelify::registration_toolbox::ICPParams icp_params;
    icp_params.inlier_distance_threshold_m = inlier_dist;
    double cloud_resolution = modelify::kInvalidCloudResolution;
    double mean_squared_distance;
    std::vector<size_t> outlier_indices;
    modelify::registration_toolbox::validateAlignment<modelify::PointSurfelType>(
        object_surfels, detection_surfels, T_object_detection.getTransformationMatrix(),
        icp_params, cloud_resolution, &mean_squared_distance, &inlier_ratio, &outlier_indices);
  } else {
    pcl::PointCloud<pcl::PointXYZ> transformed_pcl;
    Eigen::Affine3f transform_eigen((T_object_detection).inverse().getTransformationMatrix());
    pcl::transformPointCloud(detection_pointcloud, transformed_pcl, transform_eigen);

    size_t num_inlier = 0;
    double squared_distance = 0;
    for (const auto& point : transformed_pcl.points) {
      cgal::PointAndPrimitiveId ppid =
          mesh_model.getClosestTriangle(cgal::Point(point.x, point.y, point.z));
      double dist = (Eigen::Vector3d(ppid.first.x(), ppid.first.y(), ppid.first.z()) -
                     Eigen::Vector3d(point.x, point.y, point.z))
                        .norm();
      num_inlier += dist < inlier_dist;
      squared_distance += dist * dist;
    }
    inlier_ratio =
        static_cast<double>(num_inlier) / static_cast<double>(detection_pointcloud.size());
  }

  return inlier_ratio;
}

Transformation optimizeTransformation(const cgal::MeshModel& mesh_model,
                                      const pcl::PointCloud<pcl::PointXYZ>& detection_pointcloud,
                                      const Transformation& T_init) {
  std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();

  std::vector<CGAL::Simple_cartesian<double>::Triangle_3> triangles;
  triangles.reserve(mesh_model.size());
  for (const auto& id : mesh_model.getFacetIds()) {
    triangles.push_back(mesh_model.getTriangle(id));
  }
  cgal::Point centroid_cgal = CGAL::centroid(triangles.begin(), triangles.end());
  Position centroid(centroid_cgal.x(), centroid_cgal.y(), centroid_cgal.z());

  std::vector<Quaternion> rotations;
  rotations.emplace_back();
  rotations.emplace_back(0, 1, 0, 0);
  rotations.emplace_back(0, 0, 1, 0);
  rotations.emplace_back(0, 0, 0, 1);

  std::vector<Position> translations;
  translations.emplace_back();
  translations.emplace_back(0, 2 * centroid.y(), 2 * centroid.z());
  translations.emplace_back(2 * centroid.x(), 0, 2 * centroid.z());
  translations.emplace_back(2 * centroid.x(), 2 * centroid.y(), 0);

  std::vector<Transformation> transformations;
  transformations.reserve(4);
  std::vector<double> inlier_ratios;
  inlier_ratios.reserve(4);
  std::vector<double> mean_squared_distances;
  mean_squared_distances.reserve(4);
  std::vector<double> optimization_criteria;
  optimization_criteria.reserve(4);

  constexpr double inlier_dist = 0.01;
  for (size_t i = 0; i < rotations.size(); ++i) {
    transformations.emplace_back(rotations[i], translations[i]);
    pcl::PointCloud<pcl::PointXYZ> transformed_pcl;
    Eigen::Affine3f transform_eigen(
        (T_init * transformations[i]).inverse().getTransformationMatrix());
    pcl::transformPointCloud(detection_pointcloud, transformed_pcl, transform_eigen);

    size_t num_inlier = 0;
    double squared_distance = 0;
    for (const auto& point : transformed_pcl.points) {
      cgal::PointAndPrimitiveId ppid =
          mesh_model.getClosestTriangle(cgal::Point(point.x, point.y, point.z));
      double dist = (Eigen::Vector3d(ppid.first.x(), ppid.first.y(), ppid.first.z()) -
                     Eigen::Vector3d(point.x, point.y, point.z))
                        .norm();
      num_inlier += dist < inlier_dist;
      squared_distance += dist * dist;
    }
    inlier_ratios.emplace_back(static_cast<double>(num_inlier) /
                               static_cast<double>(detection_pointcloud.size()));
    mean_squared_distances.emplace_back(squared_distance /
                                        static_cast<double>(detection_pointcloud.size()));
  }
  LOG(INFO) << "Time optimization: "
            << std::chrono::duration<float>(std::chrono::steady_clock::now() - time_start).count()
            << " s";

  // Select best orientation
  const auto& most_inliers = std::max_element(inlier_ratios.begin(), inlier_ratios.end());
  // TODO(gasserl): consider other criteria
  return T_init * transformations[std::distance(inlier_ratios.begin(), most_inliers)];
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
    LOG(WARNING) << "No ICP config file given, using default ICP settings";
    icp.setDefault();
  }

  // estimate normals if necessary
  if (icp.errorMinimizer->className == "PointToPlaneErrorMinimizer") {
    modelify::PointSurfelCloudType detection_surfels = estimateNormals(detection_pointcloud);
    points_detection = convertPclToDataPoints(detection_surfels);
  }

  // icp: reference - object mesh, data - detection cloud
  PM::TransformationParameters T_object_detection_icp;
  try {
    T_object_detection_icp = icp.compute(points_object, points_detection,
                                         T_object_detection_init.getTransformationMatrix());
  } catch (PM::ConvergenceError& error_msg) {
    LOG(ERROR) << "ICP was not successful!";
    return T_object_detection_init;
  }

  if (icp.getMaxNumIterationsReached()) {
    LOG(WARNING) << "ICP reached maximum number of iterations!";
  }

  if (!Quaternion::isValidRotationMatrix(T_object_detection_icp.block<3, 3>(0, 0))) {
    for (int i = 0; i < 3; ++i) {
      T_object_detection_icp.block<3, 1>(0, i) =
          T_object_detection_icp.block<3, 1>(0, i).normalized();
    }

    if (!Quaternion::isValidRotationMatrix(T_object_detection_icp.block<3, 3>(0, 0))) {
      LOG(ERROR) << "No valid rotation matrix possible!";
      return T_object_detection_init;
    }
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
  icp.setMaximumIterations(30);
  icp.setInputSource(boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(object_pointcloud));
  icp.setInputTarget(boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(detection_pointcloud));

  pcl::PointCloud<pcl::PointXYZ> pcl;
  icp.align(pcl, T_object_detection_init.getTransformationMatrix());

  if (!icp.hasConverged()) {
    LOG(WARNING) << "ICP hasn't converged!";
  }

  Eigen::Matrix4f T_object_detection_icp = icp.getFinalTransformation();

  if (!Quaternion::isValidRotationMatrix(T_object_detection_icp.block<3, 3>(0, 0))) {
    for (int i = 0; i < 3; ++i) {
      T_object_detection_icp.block<3, 1>(0, i) =
          T_object_detection_icp.block<3, 1>(0, i).normalized();
    }

    if (!Quaternion::isValidRotationMatrix(T_object_detection_icp.block<3, 3>(0, 0))) {
      LOG(ERROR) << "No valid rotation matrix possible!";
      return T_object_detection_init;
    }
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
  for (size_t i = 0; i < points.size(); ++i) {
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

PM::DataPoints convertPclToDataPoints(const modelify::PointSurfelCloudType& pointcloud) {
  // Define feature labels
  PM::DataPoints::Labels feature_labels;
  feature_labels.push_back(PM::DataPoints::Label("x", 1));
  feature_labels.push_back(PM::DataPoints::Label("y", 1));
  feature_labels.push_back(PM::DataPoints::Label("z", 1));
  feature_labels.push_back(PM::DataPoints::Label("pad", 1));

  // Define descriptor labels
  PM::DataPoints::Labels descriptor_labels;
  descriptor_labels.push_back(PM::DataPoints::Label("normals", 3));

  // Get features and descriptors from pointcloud
  PM::Matrix features(feature_labels.totalDim(), pointcloud.size());
  PM::Matrix descriptors(descriptor_labels.totalDim(), pointcloud.size());
  std::chrono::steady_clock::time_point conversion_start = std::chrono::steady_clock::now();
  for (uint i = 0; i < pointcloud.size(); ++i) {
    features.col(i) =
        Eigen::Vector4f(pointcloud.points[i].x, pointcloud.points[i].y, pointcloud.points[i].z, 1);
    descriptors.col(i) =
        Eigen::Vector3f(pointcloud.points[i].normal_x, pointcloud.points[i].normal_y,
                        pointcloud.points[i].normal_z);
  }
  LOG(INFO)
      << "Time conversion PCL  to pointmatcher: "
      << std::chrono::duration<float>(std::chrono::steady_clock::now() - conversion_start).count()
      << " s";

  return PM::DataPoints(features, feature_labels, descriptors, descriptor_labels);
}

modelify::PointSurfelCloudType convertDataPointsToPointSurfels(const PM::DataPoints& data_points) {
  std::chrono::steady_clock::time_point conversion_start = std::chrono::steady_clock::now();

  // Get indices of coordinates and normal
  int index_x, index_y, index_z;
  int count = 0;
  for (size_t i = 0; i < data_points.featureLabels.size(); ++i) {
    if (data_points.featureLabels[i].text == "x") {
      index_x = count;
    } else if (data_points.featureLabels[i].text == "y") {
      index_y = count;
    } else if (data_points.featureLabels[i].text == "z") {
      index_z = count;
    }
    count += data_points.featureLabels[i].span;
  }
  int index_normals = -1;
  count = 0;
  for (size_t i = 0; i < data_points.descriptorLabels.size(); ++i) {
    if (data_points.descriptorLabels[i].text == "normal") {
      index_normals = count;
    }
    count += data_points.descriptorLabels[i].span;
  }
  bool use_normals = index_normals == -1;
  if (use_normals) {
    LOG(WARNING) << "Data points do not contain a normal descriptor!";
  }

  // Convert to pointcloud
  modelify::PointSurfelCloudType pointcloud;
  for (size_t i = 0; i < data_points.getNbPoints(); ++i) {
    pcl::PointSurfel point;
    point.x = data_points.features.col(i)[index_x];
    point.y = data_points.features.col(i)[index_y];
    point.z = data_points.features.col(i)[index_z];
    if (use_normals) {
      point.normal_x = data_points.descriptors.col(i)[index_normals + 0];
      point.normal_y = data_points.descriptors.col(i)[index_normals + 1];
      point.normal_z = data_points.descriptors.col(i)[index_normals + 2];
    }
    pointcloud.emplace_back(point);
  }

  LOG(INFO)
      << "Time conversion PCL to pointmatcher: "
      << std::chrono::duration<float>(std::chrono::steady_clock::now() - conversion_start).count()
      << " s";
  return pointcloud;
}

modelify::PointSurfelCloudType estimateNormals(const pcl::PointCloud<pcl::PointXYZ>& pointcloud_xyz,
                                               const cgal::MeshModel& mesh_model) {
  std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

  // Get normals from mesh
  modelify::PointSurfelCloudType pointcloud_surfels;
  for (const auto& point : pointcloud_xyz) {
    // Use normal of closest mesh triangle
    const cgal::PointAndPrimitiveId& ppid =
        mesh_model.getClosestTriangle(point.x, point.y, point.z);
    const cgal::Vector& normal = mesh_model.getNormal(ppid);

    // Add to pointcloud
    modelify::PointSurfelType surfel;
    pcl::copyPoint(point, surfel);
    surfel.normal_x = normal.x();
    surfel.normal_y = normal.y();
    surfel.normal_z = normal.z();
    pointcloud_surfels.emplace_back(surfel);
  }
  removeNanFromPointcloud(pointcloud_surfels);

  LOG(INFO) << "Time normals: "
            << std::chrono::duration<float>(std::chrono::steady_clock::now() - start).count()
            << " s";
  return pointcloud_surfels;
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
  removeNanFromPointcloud(pointcloud_surfels);

  LOG(INFO) << "Time normals: "
            << std::chrono::duration<float>(std::chrono::steady_clock::now() - start).count()
            << " s";
  return pointcloud_surfels;
}

void removeNanFromPointcloud(modelify::PointSurfelCloudType& pointcloud_surfels) {
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
}

modelify::PointSurfelCloudType computeKeypoints(
    const KeypointType& keypoint_type,
    const modelify::PointSurfelCloudType::Ptr& pointcloud_surfel_ptr) {
  CHECK(pointcloud_surfel_ptr);

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
      uniform_params.search_radius = 0.01;
      if (!modelify::feature_toolbox::getKeypointsFromUniformDownsampling(
              pointcloud_surfel_ptr, uniform_params, keypoints)) {
        LOG(WARNING) << "Could not extract uniform keypoints!";
      }
      break;
    }
    case kAll: {
      keypoints = pointcloud_surfel_ptr;
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
pcl::PointCloud<modelify::DescriptorSHOT> computeDescriptors<modelify::DescriptorSHOT>(
    const modelify::PointSurfelCloudType::Ptr& pointcloud_surfel_ptr,
    const modelify::PointSurfelCloudType::Ptr& keypoints) {
  CHECK(pointcloud_surfel_ptr);
  CHECK(keypoints);

  modelify::feature_toolbox::SHOTParams shot_params;
  shot_params.search_radius = 0.01;
  const pcl::PointCloud<modelify::DescriptorSHOT>::Ptr descriptors(
      new pcl::PointCloud<modelify::DescriptorSHOT>());
  modelify::feature_toolbox::describeKeypoints<modelify::DescriptorSHOT>(
      pointcloud_surfel_ptr, modelify::kInvalidCloudResolution, shot_params, keypoints,
      descriptors);
  return *descriptors;
}

template <>
pcl::PointCloud<modelify::DescriptorFPFH> computeDescriptors<modelify::DescriptorFPFH>(
    const modelify::PointSurfelCloudType::Ptr& pointcloud_surfel_ptr,
    const modelify::PointSurfelCloudType::Ptr& keypoints) {
  CHECK(pointcloud_surfel_ptr);
  CHECK(keypoints);

  modelify::feature_toolbox::FPFHParams fpfh_params;
  fpfh_params.search_radius = 0.015;
  const pcl::PointCloud<modelify::DescriptorFPFH>::Ptr descriptors(
      new pcl::PointCloud<modelify::DescriptorFPFH>());
  modelify::feature_toolbox::describeKeypoints<modelify::DescriptorFPFH>(
      pointcloud_surfel_ptr, modelify::kInvalidCloudResolution, fpfh_params, keypoints,
      descriptors);
  return *descriptors;
}

template <>
pcl::PointCloud<LearnedDescriptor> computeDescriptors<LearnedDescriptor>(
    const modelify::PointSurfelCloudType::Ptr& pointcloud_surfel_ptr,
    const modelify::PointSurfelCloudType::Ptr& keypoints) {
  // Parameters
  float voxel_size = 0.001;
  float kernel_size = 0.005;

  // Clean files
  std::string filename_pointcloud = "/home/laura/bags/matching/pointcloud.ply";
  std::remove(filename_pointcloud.c_str());
  std::string filename_keypoints = "/home/laura/bags/matching/keypoints.txt";
  std::remove(filename_keypoints.c_str());
  std::string filepath_input = "/home/laura/bags/matching/sdv/";
  char voxel_size_char[100];
  std::sprintf(voxel_size_char, "%.6f", voxel_size);
  std::string voxel_size_str(voxel_size_char);
  char kernel_size_char[100];
  std::sprintf(kernel_size_char, "%.6f", kernel_size);
  std::string kernel_size_str(kernel_size_char);
  std::string filename_csv =
      filepath_input + "pointcloud_" + voxel_size_str + "_16_" + kernel_size_str + ".csv";
  std::remove(filename_csv.c_str());
  std::string filepath_output = "/home/laura/bags/matching";
  std::string filename_descriptor = filepath_output + "/32_dim/pointcloud_" + voxel_size_str +
                                    "_16_" + kernel_size_str + "_3DSmoothNet.txt";
  std::remove(filename_descriptor.c_str());

  // Write to file
  LOG(INFO) << "Writing pointcloud to file " << filename_pointcloud;
  pcl::PointCloud<pcl::PointXYZ> pcl_xyz;
  pcl::copyPointCloud(*pointcloud_surfel_ptr, pcl_xyz);
  Eigen::Vector4f origin = pcl_xyz.sensor_origin_;
  Eigen::Quaternionf orientation = pcl_xyz.sensor_orientation_;
  pcl::PCLPointCloud2 blob;
  pcl::toPCLPointCloud2(pcl_xyz, blob);
  blob.row_step = blob.data.size();
  blob.width = pcl_xyz.size();
  pcl::PLYWriter ply_writer;
  ply_writer.write(filename_pointcloud, blob, origin, orientation, false, true);

  LOG(INFO) << "Writing keypoints to file " << filename_keypoints;
  std::ofstream keypoint_file;
  keypoint_file.open(filename_keypoints);
  for (const auto& keypoint : keypoints->points) {
    for (size_t idx = 0; idx < pointcloud_surfel_ptr->points.size(); ++idx) {
      if ((keypoint.x - pointcloud_surfel_ptr->points[idx].x) < 1e-6 &&
          (keypoint.y - pointcloud_surfel_ptr->points[idx].y) < 1e-6 &&
          (keypoint.z - pointcloud_surfel_ptr->points[idx].z) < 1e-6) {
        keypoint_file << idx << "\n";
        break;
      }
    }
  }
  keypoint_file.close();

  // Prep input
  std::string python_path = "/home/laura/pilot_ws/src/3DSmoothNet";
  std::string command_parametrize = python_path + "/3DSmoothNet -r " + voxel_size_str + " -h " +
                                    kernel_size_str + " -f " + filename_pointcloud + " -k " +
                                    filename_keypoints + " -o " + filepath_input;

  LOG(INFO) << "\n------------------------------------------------PYHTON------------------------"
               "-----------------------\n"
            << std::endl;
  int result_parametrize = system(command_parametrize.c_str());
  LOG(INFO) << "\n-----------------------------------------------/PYHTON------------------------"
               "-----------------------\n"
            << std::endl;

  LOG(INFO) << "Executed command: " << command_parametrize << "\nResult: " << result_parametrize;
  if (result_parametrize != 0) {
    LOG(ERROR) << "Parametrization failed!";
    return pcl::PointCloud<LearnedDescriptor>();
  }

  // Get Descriptors by inference
  std::string command_inference = "python3 " + python_path +
                                  "/main_cnn.py --run_mode=test "
                                  "--evaluate_input_folder=" +
                                  filepath_input + " --evaluate_output_folder=" + filepath_output +
                                  " --saved_model_dir=" + python_path + "/models/" +
                                  " --training_data_folder=" + python_path +
                                  "/data/train/trainingData3DMatch";

  LOG(INFO) << "\n------------------------------------------------PYHTON------------------------"
               "-----------------------\n"
            << std::endl;
  int result_inference = system(command_inference.c_str());
  LOG(INFO) << "\n-----------------------------------------------/PYHTON------------------------"
               "-----------------------\n"
            << std::endl;
  LOG(INFO) << "Executed command: " << command_inference << "\nResult: " << result_inference;
  if (result_inference != 0) {
    LOG(ERROR) << "Inference failed!";
  }

  // Read descriptors from file
  pcl::PointCloud<LearnedDescriptor> descriptors;
  std::fstream descriptor_file;
  descriptor_file.open(filename_descriptor);
  if (descriptor_file.is_open()) {
    std::string line_str;
    while (std::getline(descriptor_file, line_str)) {
      LearnedDescriptor point{};
      size_t i = 0;
      while (!line_str.empty() && i < LearnedDescriptor::descriptorSize()) {
        size_t idx = line_str.find_first_of(',');
        point.learned_descriptor[i] = std::stof(line_str.substr(0, idx));
        line_str.erase(0, idx + 1);
        ++i;
      }
      if (i != 32) {
        LOG(ERROR) << "Descriptor length " << i << " instead of 32!";
      }
      descriptors.emplace_back(point);
    }
  } else {
    LOG(ERROR) << "Could not open file " << filename_descriptor;
  }
  LOG(INFO) << "Got " << descriptors.size() << " descriptors.";
  LOG(INFO) << "For " << keypoints->size() << " keypoints";
  return descriptors;
}

template <>
pcl::PointCloud<UnitDescriptor> computeDescriptors<UnitDescriptor>(
    const modelify::PointSurfelCloudType::Ptr& /*pointcloud_surfel_ptr*/,
    const modelify::PointSurfelCloudType::Ptr& keypoints) {
  CHECK(keypoints);

  pcl::PointCloud<UnitDescriptor> descriptors;
  for (size_t i = 0; i < keypoints->size(); ++i) {
    descriptors.points.emplace_back(UnitDescriptor());
  }
  return descriptors;
}

template <>
modelify::CorrespondencesType computeCorrespondences<UnitDescriptor>(
    const modelify::PointSurfelCloudType::Ptr& detection_keypoints,
    const typename pcl::PointCloud<UnitDescriptor>::Ptr& /*detection_descriptors*/,
    const modelify::PointSurfelCloudType::Ptr& object_keypoints,
    const typename pcl::PointCloud<UnitDescriptor>::Ptr& /*object_descriptors*/,
    double /*similarity_threshold*/) {
  CHECK(detection_keypoints);
  CHECK(object_keypoints);

  LOG(INFO) << "Computing correspondence for uniform descriptor";
  modelify::CorrespondencesTypePtr correspondences(new modelify::CorrespondencesType());

  // Find all possible correspondences
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  correspondences->reserve(object_keypoints->size() * detection_keypoints->size());
  for (size_t idx_object = 0; idx_object < object_keypoints->size(); ++idx_object) {
    for (size_t idx_detection = 0; idx_detection < detection_keypoints->size(); ++idx_detection) {
      pcl::Correspondence corr(idx_object, idx_detection, 0);
      correspondences->emplace_back(corr);
    }
  }
  LOG(INFO) << "Computed " << correspondences->size() << " correspondences.";

  // Select number of correspondences
  const size_t max_correspondences = correspondences->size();
  if (correspondences->size() > max_correspondences) {
    if (correspondences->size() - max_correspondences > max_correspondences) {
      modelify::CorrespondencesType corr_filtered;
      for (size_t i = 0; i < max_correspondences; ++i) {
        int idx = std::rand() / RAND_MAX;
        corr_filtered.emplace_back(correspondences->at(idx));
      }
      *correspondences = corr_filtered;
    } else {
      while (correspondences->size() > max_correspondences) {
        int idx = std::rand() / RAND_MAX;
        correspondences->erase(correspondences->begin() + idx);
      }
    }
    LOG(INFO) << "Reduced to " << correspondences->size() << " correspondences.";
  }

  LOG(INFO) << "Time correspondences: "
            << std::chrono::duration<float>(std::chrono::steady_clock::now() - begin).count()
            << " s";
  return *correspondences;
}

}  // namespace cad_percept::object_detection