#include "cpt_object_detection/object_detection.h"

#include <CGAL/linear_least_squares_fitting_3.h>
#include <cpt_utils/cpt_utils.h>
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
  ros::WallTime conversion_start = ros::WallTime::now();
  for (const auto& id : mesh_model->getFacetIds()) {
    CGAL::Simple_cartesian<double>::Triangle_3 triangle = mesh_model->getTriangle(id);
    CGAL::Simple_cartesian<double>::Point_3 centroid =
        CGAL::centroid(triangle);  // TODO(gasserl): replace with just a vertex?
    CGAL::Simple_cartesian<double>::Vector_3 normal =
        triangle.supporting_plane().orthogonal_vector();

    features.col(i) = Eigen::Vector4f(centroid.x(), centroid.y(), centroid.z(), 1);
    descriptors.col(i) = Eigen::Vector3f(normal.x(), normal.y(), normal.z());
    ++i;
  }
  LOG(INFO) << "Time conversion mesh to pointmatcher: "
            << (ros::WallTime::now() - conversion_start).toSec() << " s";

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

}  // namespace object_detection
}  // namespace cad_percept