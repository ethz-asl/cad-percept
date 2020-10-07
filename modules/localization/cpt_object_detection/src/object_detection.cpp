#include "cpt_object_detection/object_detection.h"

#include <CGAL/linear_least_squares_fitting_3.h>
#include <cpt_object_detection/learned_descriptor.h>
#include <cpt_utils/cpt_utils.h>
#include <modelify/feature_toolbox/descriptor_toolbox_3d.h>
#include <modelify/feature_toolbox/keypoint_toolbox_3d.h>
#include <modelify/registration_toolbox/registration_toolbox.h>
#include <pcl/common/pca.h>

#include <thread>

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
  std::chrono::steady_clock::time_point conversion_start = std::chrono::steady_clock::now();
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

template <>
void getDescriptors<LearnedDescriptor>(
    const modelify::PointSurfelCloudType::Ptr& pointcloud_surfel_ptr,
    const modelify::PointSurfelCloudType::Ptr& keypoints,
    const pcl::PointCloud<LearnedDescriptor>::Ptr& descriptors) {
  // Write to file
  std::string filename_pointcloud = "/home/laura/bags/matching/pointcloud.ply";
  LOG(INFO) << "Writing pointcloud to file " << filename_pointcloud;
  pcl::PLYWriter ply_writer;
  ply_writer.write(filename_pointcloud, *pointcloud_surfel_ptr);

  std::string filename_keypoints = "/home/laura/bags/matching/keypoints.txt";
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
  //  system("cd /home/laura/pilot_ws/src/3DSmoothNet");
  float radius = 0.002;
  char radius_char[100];
  std::sprintf(radius_char, "%.6f", radius);
  std::string radius_str(radius_char);
  std::string filepath_input = "/home/laura/bags/matching/sdv/";
  std::string args_parametrize = "./3DSmoothNet -r " + radius_str + " -f " + filename_pointcloud +
                                 " -k " + filename_keypoints + " -o " + filepath_input;
  std::string command_parametrize = "gnome-terminal -x sh -c '" + args_parametrize + "'";
  command_parametrize = args_parametrize;
  //  system(command_parametrize.c_str());
  //  LOG(INFO) << "Executed command \"" << command_parametrize << "\"";

  // Get Descriptors by inference
  std::string filepath_output = "/home/laura/bags/matching";
  std::string args_inference =
      "python3 ./main_cnn.py --run_mode=test "
      "--evaluate_input_folder=" +
      filepath_input + " --evaluate_output_folder=" + filepath_output;
  std::string command_inference = "gnome-terminal -x sh -c '" + args_inference + "'";
  command_inference = args_inference;
  //  system(command_inference.c_str());
  //  LOG(INFO) << "Executed command \"" << command_inference << "\"";

  std::string command = ("gnome-terminal -x sh -c 'cd /home/laura/pilot_ws/src/3DSmoothNet ; " +
                         args_parametrize + " ; " + args_inference + "'");
  system(command.c_str());
  LOG(INFO) << "Executed command: " << command;

  // Wait until done in 0.5 s steps
  std::string filename_descriptor =
      filepath_output + "/32_dim/pointcloud_" + radius_str + "_16_1.750000_3DSmoothNet.txt";
  std::fstream descriptor_file;
  descriptor_file.open(filename_descriptor);
  int count = 0;
  float dt = 0.5;
  int max_count = std::ceil(30 / dt);
  while (count < max_count && !descriptor_file.is_open()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(int(std::round(dt * 1000))));
    descriptor_file.open(filename_descriptor);
    ++count;
  }
  if (count > 0) {
    LOG(INFO) << "Waited " << dt * count << " seconds.";
  }

  if (descriptor_file.is_open()) {
    std::string line_str;
    while (std::getline(descriptor_file, line_str)) {
      LearnedDescriptor point{};
      int i = 0;
      while (!line_str.empty() && i < LearnedDescriptor::descriptorSize()) {
        size_t idx = line_str.find_first_of(",");
        point.learned_descriptor[i] = std::stof(line_str.substr(0, idx));
        line_str.erase(0, idx + 1);
        ++i;
      }
      if (i != 32) {
        LOG(ERROR) << "Descriptor length " << i << " instead of 32!";
      }
      descriptors->emplace_back(point);
    }
  } else {
    LOG(ERROR) << "Could not open file " << filename_descriptor;
  }
  LOG(INFO) << "Got " << descriptors->size() << " descriptors.";

  // Clean files
  std::remove(filename_pointcloud.c_str());
  std::remove(filename_keypoints.c_str());
  std::remove(filename_descriptor.c_str());
}

}  // namespace object_detection
}  // namespace cad_percept