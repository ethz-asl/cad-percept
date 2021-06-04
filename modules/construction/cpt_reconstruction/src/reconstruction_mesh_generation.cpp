#include "cpt_reconstruction/reconstruction_mesh_generation.h"
#include "cpt_reconstruction/reconstruction_proposal_selection.h"

namespace cad_percept {
namespace cpt_reconstruction {
MeshGeneration::MeshGeneration(ros::NodeHandle nodeHandle1,
                               ros::NodeHandle nodeHandle2)
    : nodeHandle1_(nodeHandle1),
      nodeHandle2_(nodeHandle2),
      vertices_model_only_(new pcl::PointCloud<pcl::PointXYZ>()),
      vertices_model_(new pcl::PointCloud<pcl::PointXYZ>()),
      upsampled_model_(new pcl::PointCloud<pcl::PointXYZ>()),
      model_upsampled_kdtree_(new pcl::search::KdTree<pcl::PointXYZ>()) {
  nodeHandle1.getParam("UpsampledBuildingModelFile",
                       UPSAMPLED_BUILDING_MODEL_PATH_);
  nodeHandle1.getParam("BuildingModelMeshFile", BUILDING_MODEL_PATH_);
  nodeHandle1.getParam("OutputMeshingTempPath", OUTPUT_DIR_);
  nodeHandle1.getParam("UpsampledModelOctreeResolution",
                       UPSAMPLED_MODEL_OCTREE_RESOLUTION_);
  nodeHandle1.getParam("MinArea", MIN_AREA_);
  nodeHandle1.getParam("DefaultOffset", DEFAULT_OFFSET_);
  nodeHandle1.getParam("DuplicateDotProduct", DUPLICATE_DOT_PRODUCT_);
  nodeHandle1.getParam("DuplicateDiffD", DUPLICATE_DIFF_D_);

  model_octree_.reset(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(
      UPSAMPLED_MODEL_OCTREE_RESOLUTION_));

  pcl::io::loadPolygonFilePLY(BUILDING_MODEL_PATH_, mesh_model_);

  pcl::fromPCLPointCloud2(mesh_model_.cloud, *vertices_model_only_);
  faces_model_only_ = mesh_model_.polygons;

  pcl::PLYReader reader;
  reader.read(UPSAMPLED_BUILDING_MODEL_PATH_, *upsampled_model_);
  model_octree_->setInputCloud(upsampled_model_);
  model_octree_->addPointsFromInputCloud();
  model_upsampled_kdtree_->setInputCloud(upsampled_model_);

  subscriber_ = nodeHandle1_.subscribe("classified_shapes", 1000,
                                       &MeshGeneration::messageCallback, this);

  publisher_ = nodeHandle2_.advertise<::cpt_reconstruction::element_proposals>(
      "element_proposals", 1000);
  ros::spin();
}

void MeshGeneration::messageCallback(
    const ::cpt_reconstruction::classified_shapes &msg) {
  double min_area = 0.0;

  // Consindering Planar Elements only
  pcl::PolygonMesh mesh_detected;
  getMessageData(msg, mesh_detected);
  preprocessFusedMesh(mesh_detected, min_area);
  getProposalVerticesPlanes(min_area);

  // Split point cloud in three parts:
  // 1. Points which are matching the vertices in the building model (strong
  // points)
  // 2. Points computed multiple times (weak points)
  // 3. Remaining points (backup)
  pcl::PointCloud<pcl::PointXYZ>::Ptr strong_points(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr weak_points(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr backup_points(
      new pcl::PointCloud<pcl::PointXYZ>);
  getHierarchicalVertices(strong_points, weak_points, backup_points);

  pcl::io::savePLYFile(OUTPUT_DIR_ + "strong_points.ply", *strong_points);
  pcl::io::savePLYFile(OUTPUT_DIR_ + "weak_points.ply", *weak_points);
  pcl::io::savePLYFile(OUTPUT_DIR_ + "backup_points.ply", *backup_points);

  //Compute Proposals from Planes
  std::vector<Eigen::Vector3d> center_estimates;
  std::vector<Eigen::Matrix3d> direction_estimates;
  std::vector<std::vector<Eigen::VectorXd>> parameter_estimates;
  getElementProposalsPlanes(center_estimates, direction_estimates,
                            parameter_estimates, strong_points, weak_points);

  //Compute Proposals from Cylinders
  std::vector<Eigen::MatrixXd> bounded_axis_estimates;
  std::vector<double> radius_estimates;
  getElementProposalsCylinders(bounded_axis_estimates, radius_estimates);


  //Select a subset from proposals and forward it to the model integration node
  ProposalSelection proposalSelection(center_estimates, direction_estimates, parameter_estimates, bounded_axis_estimates, radius_estimates);
  proposalSelection.selectProposals();
  proposalSelection.getSelectedProposals();

  /*
  pcl::PolygonMesh resulting_mesh;
  evaluateProposals(resulting_mesh, center_estimates, direction_estimates,
                    parameter_estimates);

  pcl::io::savePLYFile(OUTPUT_DIR_ + "reconstructed_mesh.ply", resulting_mesh);
  */
}

void MeshGeneration::preprocessFusedMesh(pcl::PolygonMesh &mesh_detected,
                                         double min_area) {
  pcl::PolygonMesh mesh_model(mesh_model_);
  pcl::io::savePLYFile(OUTPUT_DIR_ + "mesh_detected.ply", mesh_detected);
  combineMeshes(mesh_detected, mesh_model);

  faces_model_ = mesh_model.polygons;
  vertices_model_.reset(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromPCLPointCloud2(mesh_model.cloud, *vertices_model_);

  mesh_plane_d_.clear();
  mesh_area_.clear();
  mesh_plane_normals_.clear();

  // Get all plane parameters from model
  computeAllPlanes(false, 0.1);

  // Flag duplicated planes
  duplicated_faces_.clear();
  flagDuplicatedPlanes(min_area);
}

void MeshGeneration::getMessageData(
    const ::cpt_reconstruction::classified_shapes &msg,
    pcl::PolygonMesh &mesh_detected) {
  meshing_clouds_.clear();
  meshing_classes_.clear();
  robot_positions_.clear();
  plane_normals_.clear();
  ids_.clear();
  axis_.clear();
  radius_.clear();
  for (int i = 0; i < msg.id.size(); i++) {
    // Get current cloud from msg
    sensor_msgs::PointCloud2 msg_cloud = msg.clouds.at(i);
    pcl::PCLPointCloud2 cloud_temp;
    pcl_conversions::toPCL(msg_cloud, cloud_temp);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cur_cloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromPCLPointCloud2(cloud_temp, *cur_cloud);

    // Get current class from msg
    int cur_class = msg.classes.at(i);

    // Get current robot position from msg
    geometry_msgs::Vector3 robot_position_temp = msg.robot_positions.at(i);
    Eigen::Vector3d robot_position(robot_position_temp.x, robot_position_temp.y,
                                   robot_position_temp.z);

    // Get current plane normal from msg
    geometry_msgs::Vector3 plane_normal_temp = msg.ransac_normal.at(i);
    Eigen::Vector3d plane_normal(plane_normal_temp.x, plane_normal_temp.y,
                                 plane_normal_temp.z);

    // Get current id from msg
    int cur_id = msg.id.at(i);

    // Get current axis from msg
    geometry_msgs::Vector3 axis_temp = msg.axis.at(i);
    Eigen::Vector3d cur_axis(axis_temp.x, axis_temp.y, axis_temp.z);

    // Get current radius from msg
    double cur_radius = msg.radius.at(i);

    bool valid_class = checkShapeConstraints(cur_class, plane_normal, cur_id);

    if (valid_class) {
      // Register data of valid shapes

      meshing_clouds_.push_back(cur_cloud);
      meshing_classes_.push_back(cur_class);
      robot_positions_.push_back(robot_position);
      plane_normals_.push_back(plane_normal);
      ids_.push_back(cur_id);
      axis_.push_back(cur_axis);
      radius_.push_back(cur_radius);

      // Consider Planar elements only
      if (cur_id == 0) {
        pcl::PolygonMesh mesh;
        computePlanarConvexHull(cur_cloud, mesh, true);
        combineMeshes(mesh, mesh_detected);
      }
    }
  }
}

void MeshGeneration::getProposalVerticesPlanes(double min_area) {
  artificial_vertices_vector_.clear();
  artificial_kdtrees_vector_.clear();
  normal_detected_shapes_vector_.clear();
  detected_shapes_d_vector_.clear();
  element_corners_vector_.clear();
  element_points_kdtrees_vector_.clear();
  shape_directions_.clear();
  for (int i = 0; i < meshing_clouds_.size(); i++) {
    // Ignore columns
    if (ids_.at(i) == 1) {
      continue;
    }

    // Some temporary variables
    pcl::PointCloud<pcl::PointXYZ>::Ptr element_points = meshing_clouds_.at(i);
    pcl::PointCloud<pcl::PointXYZ>::Ptr corners(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr corners_top_bottom(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr corners_side(
        new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Vector3d element_normal;
    double element_d;

    processElementCloud(i, element_points, corners, corners_top_bottom,
                        corners_side, element_normal, element_d);

    normal_detected_shapes_vector_.push_back(element_normal);
    detected_shapes_d_vector_.push_back(element_d);
    element_corners_vector_.push_back(corners);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr element_kd_tree(
        new pcl::search::KdTree<pcl::PointXYZ>());
    element_kd_tree->setInputCloud(element_points);
    element_points_kdtrees_vector_.push_back(element_kd_tree);

    // Candidates: local scope
    candidate_faces_main_.clear();
    candidate_faces_ortho_horizontal_.clear();
    candidate_faces_ortho_vertical_.clear();
    selectMainCandidateFaces(corners, element_normal, min_area, true);

    // Add planes with orthogonal normal and in local scope
    if (meshing_classes_.at(i) == Semantics::WALL ||
        meshing_classes_.at(i) == Semantics::BEAM) {
      selectOrthoCandidateFacesWall(corners_side, corners_top_bottom,
                                    element_normal, min_area);
    } else if (meshing_classes_.at(i) == Semantics::FLOOR ||
               meshing_classes_.at(i) == Semantics::CEILING) {
      selectOrthoCandidateFacesFloorCeiling(corners, element_normal, min_area);
    } else {
      ROS_INFO("Class not implemented yet!");
      assert(false);
    }

    int nr_main = candidate_faces_main_.size();
    int nr_ortho_vertical = candidate_faces_ortho_vertical_.size();
    int nr_ortho_horizontal = candidate_faces_ortho_horizontal_.size();
    ROS_INFO("Class number: %d", meshing_classes_.at(i));
    ROS_INFO("Number of main: %d", nr_main);
    ROS_INFO("Number of ortho_vertical: %d", nr_ortho_vertical);
    ROS_INFO("Number of ortho_horizontal: %d", nr_ortho_horizontal);

    if (nr_main == 0 || nr_ortho_vertical == 0 || nr_ortho_horizontal == 0) {
      ROS_INFO("Missing a direction -> skip");

      // Don't mess up index structure!!
      pcl::PointCloud<pcl::PointXYZ>::Ptr artificial_vertices(
          new pcl::PointCloud<pcl::PointXYZ>);
      // TODO - Find better solution for placeholder!
      artificial_vertices->push_back(pcl::PointXYZ(0, 0, -3));
      artificial_vertices_vector_.push_back(artificial_vertices);
      pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree(
          new pcl::search::KdTree<pcl::PointXYZ>());
      kd_tree->setInputCloud(artificial_vertices);
      artificial_kdtrees_vector_.push_back(kd_tree);
      shape_directions_.push_back(Eigen::Matrix3d::Identity());
      continue;
    }

    // Compute all intersections directly
    pcl::PointCloud<pcl::PointXYZ>::Ptr artificial_vertices(
        new pcl::PointCloud<pcl::PointXYZ>);
    computeArtificialVertices(artificial_vertices, shape_directions_);
    artificial_vertices_vector_.push_back(artificial_vertices);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree(
        new pcl::search::KdTree<pcl::PointXYZ>());
    kd_tree->setInputCloud(artificial_vertices);
    artificial_kdtrees_vector_.push_back(kd_tree);
  }
}

void MeshGeneration::getHierarchicalVertices(
    pcl::PointCloud<pcl::PointXYZ>::Ptr strong_points,
    pcl::PointCloud<pcl::PointXYZ>::Ptr weak_points,
    pcl::PointCloud<pcl::PointXYZ>::Ptr backup_points) {
  std::vector<int> nn_indices{1};
  std::vector<float> nn_dists{1};
  for (int i = 0; i < vertices_model_only_->size(); i++) {
    pcl::PointXYZ p_model = (*vertices_model_only_)[i];
    for (int j = 0; j < artificial_kdtrees_vector_.size(); j++) {
      artificial_kdtrees_vector_.at(j)->nearestKSearch(p_model, 1, nn_indices,
                                                       nn_dists);
      if (std::sqrt(nn_dists[0]) < 10e-4) {
        strong_points->push_back(p_model);
        break;
      }
    }
  }

  for (int i1 = 0; i1 < artificial_vertices_vector_.size(); i1++) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cur_cloud =
        artificial_vertices_vector_.at(i1);
    for (int i2 = 0; i2 < cur_cloud->size(); i2++) {
      pcl::PointXYZ cur_point = (*cur_cloud)[i2];
      for (int j = 0; j < artificial_kdtrees_vector_.size(); j++) {
        if (i1 == j) {
          continue;
        }
        pcl::search::KdTree<pcl::PointXYZ>::Ptr cur_kd_tree =
            artificial_kdtrees_vector_.at(j);
        cur_kd_tree->nearestKSearch(cur_point, 1, nn_indices, nn_dists);
        if (std::sqrt(nn_dists[0]) < 10e-4) {
          weak_points->push_back(cur_point);
          break;
        }
      }
    }
  }

  for (int i = 0; i < artificial_vertices_vector_.size(); i++) {
    (*backup_points) += *(artificial_vertices_vector_.at(i));
  }

  // Remove duplicates
  removeDuplicatedPoints(strong_points);
  removeDuplicatedPoints(weak_points);
  removeDuplicatedPoints(backup_points);
}

void MeshGeneration::getElementProposalsPlanes(
    std::vector<Eigen::Vector3d> &center_estimates,
    std::vector<Eigen::Matrix3d> &direction_estimates,
    std::vector<std::vector<Eigen::VectorXd>> &parameter_estimates,
    pcl::PointCloud<pcl::PointXYZ>::Ptr strong_points,
    pcl::PointCloud<pcl::PointXYZ>::Ptr weak_points) {
  // reconstruction
  // Starting with elements element with the highest support of existing
  // vertices
  int max_idx = 0;
  int max_value = 0;
  std::vector<int> nn_indices{1};
  std::vector<float> nn_dists{1};
  // pcl::PolygonMesh mesh_all;
  std::vector<int> done_shapes;
  pcl::PointCloud<pcl::PointXYZ>::Ptr strong_points_reconstruction;

  do {
    for (int i = 0; i < artificial_kdtrees_vector_.size(); i++) {
      if (std::find(done_shapes.begin(), done_shapes.end(), i) !=
          done_shapes.end()) {
        continue;
      }

      int counter_model_vertices = 0;
      pcl::PointCloud<pcl::PointXYZ>::Ptr strong_points_reconstruction_temp(
          new pcl::PointCloud<pcl::PointXYZ>);
      for (int j = 0; j < strong_points->size(); j++) {
        pcl::PointXYZ p = (*strong_points)[j];
        artificial_kdtrees_vector_.at(i)->nearestKSearch(p, 1, nn_indices,
                                                         nn_dists);
        if (std::sqrt(nn_dists[0]) < 10e-4) {
          counter_model_vertices++;
          strong_points_reconstruction_temp->push_back(p);
        }
      }
      if (max_value <= counter_model_vertices) {
        max_value = counter_model_vertices;
        max_idx = i;
        strong_points_reconstruction = strong_points_reconstruction_temp;
      }
    }

    // Reconstruct i and remove it
    ROS_INFO("Element to reconstruct: %d;  number of strong points: %d",
             max_idx, max_value);

    // TODO: Weak points vs backup_points
    getReconstructionParametersPlanes(max_idx, strong_points_reconstruction,
                                      artificial_vertices_vector_.at(max_idx),
                                      artificial_vertices_vector_.at(max_idx),
                                      center_estimates, direction_estimates,
                                      parameter_estimates);

    // TODO Remove?
    /*
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_strong_points(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromPCLPointCloud2(mesh_all.cloud, *new_strong_points);
    *strong_points += *new_strong_points;
    */
    done_shapes.push_back(max_idx);
    max_idx = 0;
    max_value = 0;
  } while (done_shapes.size() < artificial_vertices_vector_.size());
}

void MeshGeneration::getElementProposalsCylinders(std::vector<Eigen::MatrixXd> &bounded_axis_estimates,
                                                  std::vector<double> &radius_estimates) {

  for(int i = 0; i < meshing_clouds_.size(); i++){
    if (ids_.at(i) != 1 && meshing_classes_.at(i) == Semantics::COLUMN){
      continue;
    }
    //Get Data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = meshing_clouds_.at(i);
    //Eigen::Vector3d axis = axis_.at(i);
    //double radius = radius_.at(i);

    //Estimate Parameters again (to ensure that a point on the axis is available)
    //
    // Recompute axis/randius and project/filter(??) points
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    // Estimate Normals
    pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTree(
        new pcl::search::KdTree<pcl::PointXYZ>);
    searchTree->setInputCloud(cloud);
    pcl::PointCloud<pcl::Normal>::Ptr normals(
        new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
    normalEstimator.setInputCloud(cloud);
    normalEstimator.setSearchMethod(searchTree);
    normalEstimator.setKSearch(10);
    normalEstimator.compute(*normals);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.05);
    seg.setMaxIterations(1000);
    seg.setInputCloud(cloud);
    seg.setInputNormals(normals);
    seg.segment(*inliers, *coefficients);

    Eigen::Vector3d point_on_axis(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    Eigen::Vector3d axis(coefficients->values[3], coefficients->values[4], coefficients->values[5]);
    axis.normalize();
    double radius = coefficients->values[6];

    pcl::PointXYZ center(0.0, 0.0, 0.0);
    for (int j = 0; j < cloud->size(); j++){
      pcl::PointXYZ p = (*cloud)[j];
      center.x += p.x;
      center.y += p.y;
      center.z += p.z;
    }
    center.x /= cloud->size();
    center.y /= cloud->size();
    center.z /= cloud->size();
    Eigen::Vector3d center_mean(center.x, center.y, center.z);

    Eigen::Vector3d center_to_axis_point = center_mean - point_on_axis;
    double t = center_to_axis_point.dot(axis);

    Eigen::Vector3d center_aligned(point_on_axis.x() + t * axis.x(), point_on_axis.y()+ t * axis.y(), point_on_axis.z() + t * axis.z());


    //Estimate upper and lower bound from scan
    double min_h = 1000;
    double max_h = -1000;
    double d_min = 0;
    double d_max = 0;
    for (int j = 0; j < cloud->size(); j++){
      pcl::PointXYZ p = (*cloud)[j];
      Eigen::Vector3d p_e(p.x, p.y, p.z);
      Eigen::Vector3d diff = p_e - center_aligned;

      double dot = axis.dot(diff);
      if (dot > max_h){
        max_h = dot;
        d_max = -(axis.x() * p.x + axis.y() * p.y + axis.z() * p.z);
      }
      if (dot < min_h){
        min_h = dot;
        d_min = -(axis.x() * p.x + axis.y() * p.y + axis.z() * p.z);
      }
    }

    //TODO: Raytracing

    //Line plane Intersection
    double t1 = -(d_min + center_aligned.dot(axis));
    double t2 = -(d_max + center_aligned.dot(axis));

    Eigen::Matrix<double, 3, 2> bounding_points;
    bounding_points.col(0) = center_aligned + t1 * axis;
    bounding_points.col(1) = center_aligned + t2 * axis;

    bounded_axis_estimates.push_back(bounding_points);
    radius_estimates.push_back(radius);
  }
}

void MeshGeneration::evaluateProposals(
    pcl::PolygonMesh &resulting_mesh,
    std::vector<Eigen::Vector3d> &center_estimates,
    std::vector<Eigen::Matrix3d> &direction_estimates,
    std::vector<std::vector<Eigen::VectorXd>> &parameter_estimates) {
  int id_counter = 0;
  std::vector<int> ids;
  std::vector<geometry_msgs::Vector3> dir_1;
  std::vector<geometry_msgs::Vector3> dir_2;
  std::vector<geometry_msgs::Vector3> dir_3;
  std::vector<geometry_msgs::Vector3> centers;
  std::vector<::cpt_reconstruction::parameters> magnitudes;

  for (int i = 0; i < center_estimates.size(); i++) {
    Eigen::Vector3d center_estimate = center_estimates.at(i);
    Eigen::Matrix3d direction_estimate = direction_estimates.at(i);
    std::vector<Eigen::VectorXd> parameter_estimate = parameter_estimates.at(i);

    Eigen::VectorXd a1 = parameter_estimate.at(0);
    Eigen::VectorXd a2 = parameter_estimate.at(1);
    Eigen::VectorXd b1 = parameter_estimate.at(2);
    Eigen::VectorXd b2 = parameter_estimate.at(3);
    Eigen::VectorXd c1 = parameter_estimate.at(4);
    Eigen::VectorXd c2 = parameter_estimate.at(5);

    ROS_INFO("a1 size: %d", a1.size());
    ROS_INFO("a2 size: %d", a2.size());
    ROS_INFO("b1 size: %d", b1.size());
    ROS_INFO("b2 size: %d", b2.size());
    ROS_INFO("c1 size: %d", c1.size());
    ROS_INFO("c2 size: %d", c2.size());

    if (a1.size() == 0 || a2.size() == 0 || b1.size() == 0 || b2.size() == 0 ||
        c1.size() == 0 || c2.size() == 0) {
      ROS_INFO("Skipped element %d", i);
      continue;
    }

    // Writing message
    ::cpt_reconstruction::parameters parameters;
    std::vector<double> a1_temp_vec(&a1[0], a1.data() + a1.cols() * a1.rows());
    std::vector<double> a2_temp_vec(&a2[0], a2.data() + a2.cols() * a2.rows());
    std::vector<double> b1_temp_vec(&b1[0], b1.data() + b1.cols() * b1.rows());
    std::vector<double> b2_temp_vec(&b2[0], b2.data() + b2.cols() * b2.rows());
    std::vector<double> c1_temp_vec(&c1[0], c1.data() + c1.cols() * c1.rows());
    std::vector<double> c2_temp_vec(&c2[0], c2.data() + c2.cols() * c2.rows());
    parameters.a1 = a1_temp_vec;
    parameters.a2 = a2_temp_vec;
    parameters.b1 = b1_temp_vec;
    parameters.b2 = b2_temp_vec;
    parameters.c1 = c1_temp_vec;
    parameters.c2 = c2_temp_vec;

    ids.push_back(id_counter++);
    magnitudes.push_back(parameters);

    geometry_msgs::Vector3 cur_dir_1;
    geometry_msgs::Vector3 cur_dir_2;
    geometry_msgs::Vector3 cur_dir_3;
    cur_dir_1.x = direction_estimate.col(0).x();
    cur_dir_1.y = direction_estimate.col(0).y();
    cur_dir_1.z = direction_estimate.col(0).z();
    cur_dir_2.x = direction_estimate.col(1).x();
    cur_dir_2.y = direction_estimate.col(1).y();
    cur_dir_2.z = direction_estimate.col(1).z();
    cur_dir_3.x = direction_estimate.col(2).x();
    cur_dir_3.y = direction_estimate.col(2).y();
    cur_dir_3.z = direction_estimate.col(2).z();
    dir_1.push_back(cur_dir_1);
    dir_2.push_back(cur_dir_2);
    dir_3.push_back(cur_dir_3);

    geometry_msgs::Vector3 cur_center;
    cur_center.x = center_estimate.x();
    cur_center.y = center_estimate.y();
    cur_center.z = center_estimate.z();
    centers.push_back(cur_center);

    double a1_max = a1.maxCoeff() > std::fabs(a1.minCoeff()) ? a1.maxCoeff()
                                                             : a1.minCoeff();
    double a2_max = a2.maxCoeff() > std::fabs(a2.minCoeff()) ? a2.maxCoeff()
                                                             : a2.minCoeff();
    double b1_max = b1.maxCoeff() > std::fabs(b1.minCoeff()) ? b1.maxCoeff()
                                                             : b1.minCoeff();
    double b2_max = b2.maxCoeff() > std::fabs(b2.minCoeff()) ? b2.maxCoeff()
                                                             : b2.minCoeff();
    double c1_max = c1.maxCoeff() > std::fabs(c1.minCoeff()) ? c1.maxCoeff()
                                                             : c1.minCoeff();
    double c2_max = c2.maxCoeff() > std::fabs(c2.minCoeff()) ? c2.maxCoeff()
                                                             : c2.minCoeff();

    ROS_INFO("a1_max %f", a1_max);
    ROS_INFO("a2_max %f", a2_max);
    ROS_INFO("b1_max %f", b1_max);
    ROS_INFO("b2_max %f", b2_max);
    ROS_INFO("c1_max %f", c1_max);
    ROS_INFO("c2_max %f", c2_max);

    Eigen::Vector3d p1 = center_estimate + direction_estimate.col(0) * a1_max +
                         direction_estimate.col(1) * b1_max +
                         direction_estimate.col(2) * c1_max;

    Eigen::Vector3d p2 = center_estimate + direction_estimate.col(0) * a1_max +
                         direction_estimate.col(1) * b1_max +
                         direction_estimate.col(2) * c2_max;

    Eigen::Vector3d p3 = center_estimate + direction_estimate.col(0) * a1_max +
                         direction_estimate.col(1) * b2_max +
                         direction_estimate.col(2) * c1_max;

    Eigen::Vector3d p4 = center_estimate + direction_estimate.col(0) * a1_max +
                         direction_estimate.col(1) * b2_max +
                         direction_estimate.col(2) * c2_max;

    Eigen::Vector3d p5 = center_estimate + direction_estimate.col(0) * a2_max +
                         direction_estimate.col(1) * b1_max +
                         direction_estimate.col(2) * c1_max;

    Eigen::Vector3d p6 = center_estimate + direction_estimate.col(0) * a2_max +
                         direction_estimate.col(1) * b1_max +
                         direction_estimate.col(2) * c2_max;

    Eigen::Vector3d p7 = center_estimate + direction_estimate.col(0) * a2_max +
                         direction_estimate.col(1) * b2_max +
                         direction_estimate.col(2) * c1_max;

    Eigen::Vector3d p8 = center_estimate + direction_estimate.col(0) * a2_max +
                         direction_estimate.col(1) * b2_max +
                         direction_estimate.col(2) * c2_max;

    pcl::PointCloud<pcl::PointXYZ>::Ptr element(
        new pcl::PointCloud<pcl::PointXYZ>());
    element->push_back(pcl::PointXYZ(p1.x(), p1.y(), p1.z()));
    element->push_back(pcl::PointXYZ(p2.x(), p2.y(), p2.z()));
    element->push_back(pcl::PointXYZ(p3.x(), p3.y(), p3.z()));
    element->push_back(pcl::PointXYZ(p4.x(), p4.y(), p4.z()));
    element->push_back(pcl::PointXYZ(p5.x(), p5.y(), p5.z()));
    element->push_back(pcl::PointXYZ(p6.x(), p6.y(), p6.z()));
    element->push_back(pcl::PointXYZ(p7.x(), p7.y(), p7.z()));
    element->push_back(pcl::PointXYZ(p8.x(), p8.y(), p8.z()));

    std::string save =
        OUTPUT_DIR_ + "final_meshing_points" + std::to_string(i) + ".ply";
    pcl::io::savePLYFile(save, *element);

    try {
      pcl::PolygonMesh reconstructed_mesh;

      pcl::PCLPointCloud2 meshing_points;
      pcl::toPCLPointCloud2(*element, meshing_points);

      reconstructed_mesh.cloud = meshing_points;
      std::vector<::pcl::Vertices> polygons;

      // a1
      ::pcl::Vertices f1;
      f1.vertices.push_back(0);
      f1.vertices.push_back(1);
      f1.vertices.push_back(3);
      f1.vertices.push_back(2);

      // a2
      ::pcl::Vertices f2;
      f2.vertices.push_back(4);
      f2.vertices.push_back(5);
      f2.vertices.push_back(7);
      f2.vertices.push_back(6);

      // b1
      ::pcl::Vertices f3;
      f3.vertices.push_back(0);
      f3.vertices.push_back(1);
      f3.vertices.push_back(5);
      f3.vertices.push_back(4);

      // b2
      ::pcl::Vertices f4;
      f4.vertices.push_back(2);
      f4.vertices.push_back(3);
      f4.vertices.push_back(7);
      f4.vertices.push_back(6);

      // c1
      ::pcl::Vertices f5;
      f5.vertices.push_back(0);
      f5.vertices.push_back(2);
      f5.vertices.push_back(6);
      f5.vertices.push_back(4);

      // c2
      ::pcl::Vertices f6;
      f6.vertices.push_back(1);
      f6.vertices.push_back(3);
      f6.vertices.push_back(7);
      f6.vertices.push_back(5);

      polygons.push_back(f1);
      polygons.push_back(f2);
      polygons.push_back(f3);
      polygons.push_back(f4);
      polygons.push_back(f5);
      polygons.push_back(f6);

      reconstructed_mesh.polygons = polygons;

      // pcl::ConvexHull<pcl::PointXYZ> chull;
      // chull.setInputCloud(element);
      // chull.setDimension(3);
      // chull.reconstruct(reconstructed_mesh);

      combineMeshes(reconstructed_mesh, resulting_mesh);
    } catch (...) {
      ROS_INFO(" Couldn't reconstruct convex hull");
    }
  }

  ::cpt_reconstruction::element_proposals element_proposals;
  element_proposals.ids = ids;
  element_proposals.magnitudes = magnitudes;
  element_proposals.centers = centers;
  element_proposals.dir_1 = dir_1;
  element_proposals.dir_2 = dir_2;
  element_proposals.dir_3 = dir_3;

  publisher_.publish(element_proposals);
  ROS_INFO("All done");
}



bool MeshGeneration::checkShapeConstraints(int sem_class,
                                           Eigen::Vector3d &normal,
                                           int cur_id) {
  if (cur_id == 0 && (sem_class == Semantics::WALL || Semantics::BEAM)) {
    if (std::fabs(normal.z()) < 0.1) {
      return true;
    } else {
      return false;
    }
  } else if (cur_id == 0 && (sem_class == Semantics::FLOOR ||
                             sem_class == Semantics::CEILING)) {
    if (std::fabs(normal.z()) > 0.9) {
      return true;
    } else {
      return false;
    }
  } else if (cur_id == 1 && sem_class == Semantics::COLUMN) {
    return true;
  } else {
    return false;
  }
}

void MeshGeneration::combineMeshes(const pcl::PolygonMesh &mesh,
                                   pcl::PolygonMesh &mesh_all) {
  // pcl::PolygonMesh::concatenate(mesh_all, mesh); ???
  // Source:
  // https://github.com/PointCloudLibrary/pcl/blob/master/common/include/pcl/PolygonMesh.h
  mesh_all.header.stamp = std::max(mesh_all.header.stamp, mesh.header.stamp);

  const auto point_offset = mesh_all.cloud.width * mesh_all.cloud.height;

  // Transform them to PointXYZ and back to PCLPointCloud (to remove redundant
  // fields)
  pcl::PointCloud<pcl::PointXYZ> mesh_all_cloud;
  pcl::PointCloud<pcl::PointXYZ> mesh_cloud;
  pcl::fromPCLPointCloud2(mesh_all.cloud, mesh_all_cloud);
  pcl::fromPCLPointCloud2(mesh.cloud, mesh_cloud);
  pcl::PCLPointCloud2 mesh_all_cloud2;
  pcl::PCLPointCloud2 mesh_cloud2;
  pcl::toPCLPointCloud2(mesh_cloud, mesh_cloud2);
  pcl::toPCLPointCloud2(mesh_all_cloud, mesh_all_cloud2);

  // Conatenate point clouds
  pcl::PCLPointCloud2 new_cloud;
  pcl::concatenatePointCloud(mesh_all_cloud2, mesh_cloud2, new_cloud);
  mesh_all.cloud = new_cloud;

  // Concatenate polygons
  std::transform(
      mesh.polygons.begin(), mesh.polygons.end(),
      std::back_inserter(mesh_all.polygons), [point_offset](auto polygon) {
        std::transform(polygon.vertices.begin(), polygon.vertices.end(),
                       polygon.vertices.begin(),
                       [point_offset](auto &point_idx) {
                         return point_idx + point_offset;
                       });
        return polygon;
      });
}

void MeshGeneration::computePlanarConvexHull(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PolygonMesh &mesh,
    bool include_offsets) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_copy(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*cloud, *cloud_copy);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_new(
      new pcl::PointCloud<pcl::PointXYZ>);

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.03);
  seg.setMaxIterations(1000);
  seg.setInputCloud(cloud_copy);
  seg.segment(*inliers, *coefficients);

  extract.setInputCloud(cloud_copy);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*cloud_new);

  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  searchTree->setInputCloud(cloud_new);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
  normalEstimator.setInputCloud(cloud_new);
  normalEstimator.setSearchMethod(searchTree);
  normalEstimator.setKSearch(10);
  normalEstimator.compute(*normals);

  pcl::PointCloud<pcl::PointXYZI>::Ptr corners(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal> harris;
  harris.setInputCloud(cloud_new);
  harris.setNonMaxSupression(false);
  harris.setRadius(0.2f);
  harris.setThreshold(0.05f);
  harris.setNormals(normals);
  harris.compute(*corners);

  pcl::PointCloud<pcl::PointXYZ>::Ptr corners_no_i(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*corners, *corners_no_i);
  Eigen::Vector3d plane_normal(coefficients->values[0], coefficients->values[1],
                               coefficients->values[2]);
  plane_normal.normalize();

  if (include_offsets) {
    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setInputCloud(corners_no_i);
    chull.reconstruct(mesh);

    pcl::PointCloud<pcl::PointXYZ>::Ptr hull_points(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromPCLPointCloud2(mesh.cloud, *hull_points);

    pcl::PointCloud<pcl::PointXYZ>::Ptr corners_shifted(
        new pcl::PointCloud<pcl::PointXYZ>());

    Eigen::Vector3d offset = plane_normal * 0.2;
    for (int i = 0; i < hull_points->size(); i++) {
      pcl::PointXYZ p = (*hull_points)[i];
      corners_shifted->push_back(
          pcl::PointXYZ(p.x + offset.x(), p.y + offset.y(), p.z + offset.z()));
      corners_shifted->push_back(
          pcl::PointXYZ(p.x - offset.x(), p.y - offset.y(), p.z - offset.z()));
    }
    pcl::PolygonMesh mesh_offsets;
    pcl::ConvexHull<pcl::PointXYZ> chull_offsets;
    chull_offsets.setDimension(3);
    chull_offsets.setInputCloud(corners_shifted);
    chull_offsets.reconstruct(mesh_offsets);

    pcl::PointCloud<pcl::PointXYZ>::Ptr shifted_hull_points(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromPCLPointCloud2(mesh_offsets.cloud, *shifted_hull_points);

    // Keep only meaningful faces
    std::vector<::pcl::Vertices> polygons = mesh_offsets.polygons;
    mesh_offsets.polygons.clear();
    for (int i = 0; i < polygons.size(); i++) {
      ::pcl::Vertices vertices = polygons.at(i);
      int idx_1 = vertices.vertices.at(0);
      int idx_2 = vertices.vertices.at(1);
      int idx_3 = vertices.vertices.at(2);

      pcl::PointXYZ p1 = shifted_hull_points->at(idx_1);
      pcl::PointXYZ p2 = shifted_hull_points->at(idx_2);
      pcl::PointXYZ p3 = shifted_hull_points->at(idx_3);

      Eigen::Vector3d e1(p1.x, p1.y, p1.z);
      Eigen::Vector3d e2(p2.x, p2.y, p2.z);
      Eigen::Vector3d e3(p3.x, p3.y, p3.z);

      Eigen::Vector3d face_normal = (e1 - e2).cross((e3 - e2));
      face_normal.normalize();

      double dot_value = std::fabs(plane_normal.dot(face_normal));
      double z_abs = std::fabs(face_normal.z());
      if ((dot_value < 0.02 || dot_value > 0.999) &&
          (z_abs < 0.02 || z_abs > 0.998)) {
        mesh_offsets.polygons.push_back(vertices);
      }
    }
    combineMeshes(mesh_offsets, mesh);
  } else {
    // Compute Convex Hull
    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setInputCloud(corners_no_i);
    chull.reconstruct(mesh);
  }
}

void MeshGeneration::computeAllPlanes(bool do_normal_correction, double eps) {
  for (int i = 0; i < faces_model_.size(); i++) {
    std::vector<uint32_t> vertices = faces_model_.at(i).vertices;
    if (vertices.size() != 3) {
      std::cout << "Warning: Non tria element" << std::endl;
    }
    pcl::PointXYZ p1 = (*vertices_model_)[vertices.at(0)];
    pcl::PointXYZ p2 = (*vertices_model_)[vertices.at(1)];
    pcl::PointXYZ p3 = (*vertices_model_)[vertices.at(2)];
    Eigen::Vector3d v1(p1.x, p1.y, p1.z);
    Eigen::Vector3d v2(p2.x, p2.y, p2.z);
    Eigen::Vector3d v3(p3.x, p3.y, p3.z);
    Eigen::Vector3d n = (v2 - v1).cross(v3 - v1);
    mesh_area_.push_back(n.norm());

    // Normal correction up to 5 degree
    n.normalize();
    if (do_normal_correction) {
      for (int i = 0; i < 3; i++) {
        if (n(i) < -(1. - eps)) {
          n(i) = -1.;
        } else if (n(i) > -eps && n(i) < eps) {
          n(i) = 0.;
        } else if (n(i) > (1. - eps)) {
          n(i) = 1.;
        }
        n.normalize();
      }
    }

    mesh_plane_normals_.push_back(n);
    double d = -(n.x() * p1.x + n.y() * p1.y + n.z() * p1.z);
    mesh_plane_d_.push_back(d);
  }
}

void MeshGeneration::flagDuplicatedPlanes(double min_area) {
  for (int i = 0; i < faces_model_.size(); i++) {
    if (std::find(duplicated_faces_.begin(), duplicated_faces_.end(), i) !=
            duplicated_faces_.end() ||
        mesh_area_.at(i) < min_area) {
      continue;
    }
    Eigen::Vector3d plane_normal_1 = mesh_plane_normals_.at(i);
    double d_1 = mesh_plane_d_.at(i);
    for (int j = i + 1; j < faces_model_.size(); j++) {
      if (std::find(duplicated_faces_.begin(), duplicated_faces_.end(), j) !=
          duplicated_faces_.end()) {
        continue;
      }
      Eigen::Vector3d plane_normal_2 = mesh_plane_normals_.at(j);
      double d_2 = mesh_plane_d_.at(j);

      if (std::fabs(plane_normal_1.dot(plane_normal_2)) > 0.999 &&
          std::fabs(d_1 - d_2) < 0.02) {
        duplicated_faces_.push_back(j);
      }
    }
  }
  ROS_INFO("Duplicated Planes: %d", duplicated_faces_.size());
}

void MeshGeneration::processElementCloud(
    int idx, pcl::PointCloud<pcl::PointXYZ>::Ptr element_points,
    pcl::PointCloud<pcl::PointXYZ>::Ptr corners,
    pcl::PointCloud<pcl::PointXYZ>::Ptr corners_top_bottom,
    pcl::PointCloud<pcl::PointXYZ>::Ptr corners_side,
    Eigen::Vector3d &element_normal, double &element_d) {
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.05);
  seg.setMaxIterations(1000);
  seg.setInputCloud(element_points);
  seg.segment(*inliers, *coefficients);

  // Compute ransac normal vector
  element_normal(0) = coefficients->values[0];
  element_normal(1) = coefficients->values[1],
  element_normal(2) = coefficients->values[2];
  element_normal.normalize();

  /*
  if (std::fabs(element_normal.z()) < 0.1) {
    element_normal.z() = 0;
    element_normal.normalize();
  }
  */

  Eigen::Vector3d rp_temp = robot_positions_.at(idx);
  pcl::PointXYZ robot_position(rp_temp.x(), rp_temp.y(), rp_temp.z());

  pcl::PointXYZ mean_point(0, 0, 0);
  for (int i = 0; i < element_points->size(); i++) {
    mean_point.x += (*element_points)[i].x;
    mean_point.y += (*element_points)[i].y;
    mean_point.z += (*element_points)[i].z;
  }
  mean_point.x /= element_points->size();
  mean_point.y /= element_points->size();
  mean_point.z /= element_points->size();
  Eigen::Vector3d shape_robot_vec(mean_point.x - robot_position.x,
                                  mean_point.y - robot_position.y,
                                  mean_point.z - robot_position.z);
  shape_robot_vec.normalize();
  // Normal direction of shape is pointing into free direction
  if (shape_robot_vec.dot(element_normal) < 0.0) {
    element_normal *= -1.;
  }
  element_d =
      -(mean_point.x * element_normal.x() + mean_point.y * element_normal.y() +
        mean_point.z * element_normal.z());

  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  searchTree->setInputCloud(element_points);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
  normalEstimator.setInputCloud(element_points);
  normalEstimator.setSearchMethod(searchTree);
  normalEstimator.setKSearch(10);
  normalEstimator.compute(*normals);

  // Filter corner points out
  pcl::PointCloud<pcl::PointXYZI>::Ptr corners_intensity(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal> harris;
  harris.setInputCloud(element_points);
  harris.setNonMaxSupression(false);
  harris.setRadius(0.2f);
  harris.setThreshold(0.2f);
  harris.setNormals(normals);
  harris.compute(*corners_intensity);

  // Remove Intensity field
  // pcl::copyPointCloud(*corners_intensity, *corners);
  pcl::copyPointCloud(*corners_intensity, *corners);

  // Corners -> Corners top and bottom, Corners side
  if (std::fabs(element_normal.z()) < 0.1) {
    double max_z = -10000;
    double min_z = 10000;
    double max_xy = -10000;
    double min_xy = 10000;
    for (int i = 0; i < corners->size(); i++) {
      pcl::PointXYZ p = (*corners)[i];
      if (p.z > max_z) {
        max_z = p.z;
      }
      if (p.z < min_z) {
        min_z = p.z;
      }
      double dist = std::fabs(p.x) + std::fabs(p.y);
      if (dist > max_xy) {
        max_xy = dist;
      }
      if (dist < min_xy) {
        min_xy = dist;
      }
    }
    double hight = max_z - min_z;
    double width = max_xy - min_xy;
    double threshold_top = max_z - 0.15 * hight;
    double threshold_bottom = min_z + 0.15 * hight;
    double threshold_right = max_xy - 0.15 * width;
    double threshold_left = min_xy + 0.15 * width;

    for (int i = 0; i < corners->size(); i++) {
      pcl::PointXYZ p = (*corners)[i];
      if (p.z > threshold_top || p.z < threshold_bottom) {
        corners_top_bottom->push_back(p);
      }
      double dist = std::fabs(p.x) + std::fabs(p.y);
      if (dist > threshold_right || dist < threshold_left) {
        corners_side->push_back(p);
      }
    }
  } else {
    pcl::copyPointCloud(*corners, *corners_side);
    pcl::copyPointCloud(*corners, *corners_top_bottom);
  }
}

void MeshGeneration::selectMainCandidateFaces(
    pcl::PointCloud<pcl::PointXYZ>::Ptr corners,
    Eigen::Vector3d &element_normal, double min_area, bool plane_flipping) {
  std::vector<Eigen::Vector3d> flipped_planes;

  // Add planes with same similar normal and in local scope
  int nr_faces = faces_model_.size();  // Break cycle!!
  for (int i = 0; i < nr_faces; i++) {
    // Ignore duplicated planes or very small areas
    if (std::find(duplicated_faces_.begin(), duplicated_faces_.end(), i) !=
            duplicated_faces_.end() ||
        mesh_area_.at(i) < min_area) {
      continue;
    }

    Eigen::Vector3d candidate_normal = mesh_plane_normals_.at(i);
    double candidate_d = mesh_plane_d_.at(i);

    if (element_normal.dot(candidate_normal) < 0) {
      candidate_normal *= -1.0;
      candidate_d *= -1.0;
    }

    if (element_normal.dot(candidate_normal) > 0.995) {
      std::vector<uint32_t> vertices = faces_model_.at(i).vertices;

      bool found_candidate = false;
      double min_distance = 1000;
      for (int p_idx = 0; p_idx < corners->size(); p_idx += 5) {
        pcl::PointXYZ p = (*corners)[p_idx];
        double error = candidate_normal.x() * p.x + candidate_normal.y() * p.y +
                       candidate_normal.z() * p.z + candidate_d;
        if (error < min_distance) {
          min_distance = error;
          if (min_distance >= -0.4 && min_distance <= 0.05) {
            found_candidate = true;
            break;
          }
        }
      }

      if (found_candidate) {
        candidate_faces_main_.push_back(i);

        // Do plane flipping
        pcl::PointXYZ p1 = (*vertices_model_)[vertices.at(0)];
        pcl::PointXYZ p2 = (*vertices_model_)[vertices.at(1)];
        pcl::PointXYZ p3 = (*vertices_model_)[vertices.at(2)];
        Eigen::Vector3d v1(p1.x, p1.y, p1.z);
        Eigen::Vector3d v2(p2.x, p2.y, p2.z);
        Eigen::Vector3d v3(p3.x, p3.y, p3.z);
        Eigen::Vector3d n = (v1 - v2).cross(v3 - v2);
        n.normalize();

        Eigen::Vector3d v1_v2 = v1 - v2;
        double length =
            std::sqrt(v1_v2.x() * v1_v2.x() + v1_v2.y() * v1_v2.y());

        if (plane_flipping && std::fabs(n.z()) < 0.1 && length >= 0.1 &&
            length <= 0.5) {
          bool already_flipped = false;
          for (int k = 0; k < flipped_planes.size(); k++) {
            Eigen::Vector3d flipping_vector = flipped_planes.at(k);
            if ((flipping_vector - (length * n)).lpNorm<Eigen::Infinity>() <
                0.01) {
              already_flipped = true;
              break;
            }
          }
          if (!already_flipped) {
            std::cout << "Do plane flipping" << std::endl;
            Eigen::Vector3d p1_pos(v1 + n * length);
            Eigen::Vector3d p2_pos(v2 + n * length);
            Eigen::Vector3d p3_pos(v3 + n * length);

            Eigen::Vector3d p1_neg(v1 - n * length);
            Eigen::Vector3d p2_neg(v2 - n * length);
            Eigen::Vector3d p3_neg(v3 - n * length);

            int vertices_size = vertices_model_->size();
            vertices_model_->push_back(
                pcl::PointXYZ(p1_pos.x(), p1_pos.y(), p1_pos.z()));
            vertices_model_->push_back(
                pcl::PointXYZ(p2_pos.x(), p2_pos.y(), p2_pos.z()));
            vertices_model_->push_back(
                pcl::PointXYZ(p3_pos.x(), p3_pos.y(), p3_pos.z()));
            pcl::Vertices vertices_shift_pos;
            vertices_shift_pos.vertices.push_back(vertices_size);
            vertices_shift_pos.vertices.push_back(vertices_size + 1);
            vertices_shift_pos.vertices.push_back(vertices_size + 2);

            vertices_model_->push_back(
                pcl::PointXYZ(p1_neg.x(), p1_neg.y(), p1_neg.z()));
            vertices_model_->push_back(
                pcl::PointXYZ(p2_neg.x(), p2_neg.y(), p2_neg.z()));
            vertices_model_->push_back(
                pcl::PointXYZ(p3_neg.x(), p3_neg.y(), p3_neg.z()));
            pcl::Vertices vertices_shift_neg;
            vertices_shift_neg.vertices.push_back(vertices_size + 3);
            vertices_shift_neg.vertices.push_back(vertices_size + 4);
            vertices_shift_neg.vertices.push_back(vertices_size + 5);
            flipped_planes.push_back(n * length);

            candidate_faces_main_.push_back(faces_model_.size());
            faces_model_.push_back(vertices_shift_pos);
            candidate_faces_main_.push_back(faces_model_.size());
            faces_model_.push_back(vertices_shift_neg);

            mesh_plane_normals_.push_back(n);
            mesh_plane_normals_.push_back(n);
            mesh_area_.push_back(n.norm());
            mesh_area_.push_back(n.norm());

            double d1 =
                -(n.x() * p1_pos.x() + n.y() * p1_pos.y() + n.z() * p1_pos.z());
            double d2 =
                -(n.x() * p1_neg.x() + n.y() * p1_neg.y() + n.z() * p1_neg.z());
            mesh_plane_d_.push_back(d1);
            mesh_plane_d_.push_back(d2);
          }
        }
      }
    }
  }
}

void MeshGeneration::selectOrthoCandidateFacesWall(
    pcl::PointCloud<pcl::PointXYZ>::Ptr corners_side,
    pcl::PointCloud<pcl::PointXYZ>::Ptr corners_top_bottom,
    Eigen::Vector3d &element_normal, double min_area) {
  for (int i = 0; i < faces_model_.size(); i++) {
    // Ignore duplicated planes or very small areas
    if (std::find(duplicated_faces_.begin(), duplicated_faces_.end(), i) !=
            duplicated_faces_.end() ||
        mesh_area_.at(i) < min_area) {
      continue;
    }
    Eigen::Vector3d candidate_normal = mesh_plane_normals_.at(i);
    double candidate_d = mesh_plane_d_.at(i);

    if (std::fabs(element_normal.dot(candidate_normal)) < 0.05) {
      std::vector<uint32_t> vertices = faces_model_[i].vertices;
      double min_distance = 1000;
      if (std::fabs(candidate_normal.z()) < 0.15) {
        for (int p_idx = 0; p_idx < corners_side->size(); p_idx++) {
          pcl::PointXYZ p = (*corners_side)[p_idx];
          double error = std::fabs(candidate_normal.x() * p.x +
                                   candidate_normal.y() * p.y +
                                   candidate_normal.z() * p.z + candidate_d);
          if (error < min_distance) {
            min_distance = error;
          }
        }
        if (min_distance <= 0.4) {
          candidate_faces_ortho_vertical_.push_back(i);
        }
      } else if (std::fabs(candidate_normal.z()) > 0.85) {
        for (int p_idx = 0; p_idx < corners_top_bottom->size(); p_idx++) {
          pcl::PointXYZ p = (*corners_top_bottom)[p_idx];
          double error = std::fabs(candidate_normal.x() * p.x +
                                   candidate_normal.y() * p.y +
                                   candidate_normal.z() * p.z + candidate_d);
          if (error < min_distance) {
            min_distance = error;
          }
        }
        if (min_distance <= 0.4) {
          candidate_faces_ortho_horizontal_.push_back(i);
        }
      }
    }
  }
}

void MeshGeneration::selectOrthoCandidateFacesFloorCeiling(
    pcl::PointCloud<pcl::PointXYZ>::Ptr corners,
    Eigen::Vector3d &element_normal, double min_area) {
  pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud(corners);
  feature_extractor.compute();
  Eigen::Vector3f major_vector, middle_vector, minor_vector;
  Eigen::Vector3f mass_center;
  feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
  Eigen::Vector3d major = major_vector.cast<double>();
  major.normalize();

  for (int i = 0; i < faces_model_.size(); i++) {
    // Ignore duplicated planes or very small areas
    if (std::find(duplicated_faces_.begin(), duplicated_faces_.end(), i) !=
            duplicated_faces_.end() ||
        mesh_area_.at(i) < min_area) {
      continue;
    }
    Eigen::Vector3d candidate_normal = mesh_plane_normals_.at(i);
    double candidate_d = mesh_plane_d_.at(i);

    if (std::fabs(element_normal.dot(candidate_normal)) < 0.05 &&
        std::fabs(candidate_normal.z()) < 0.15) {
      std::vector<uint32_t> vertices = faces_model_[i].vertices;
      double min_distance = 1000;
      if (std::fabs(candidate_normal.dot(major)) > 0.98) {
        for (int p_idx = 0; p_idx < corners->size(); p_idx++) {
          pcl::PointXYZ p = (*corners)[p_idx];
          double error = std::fabs(candidate_normal.x() * p.x +
                                   candidate_normal.y() * p.y +
                                   candidate_normal.z() * p.z + candidate_d);
          if (error < min_distance) {
            min_distance = error;
          }
        }
        if (min_distance <= 0.4) {
          candidate_faces_ortho_vertical_.push_back(i);
        }
      } else if (std::fabs(candidate_normal.dot(major)) < 0.15) {
        for (int p_idx = 0; p_idx < corners->size(); p_idx++) {
          pcl::PointXYZ p = (*corners)[p_idx];
          double error = std::fabs(candidate_normal.x() * p.x +
                                   candidate_normal.y() * p.y +
                                   candidate_normal.z() * p.z + candidate_d);
          if (error < min_distance) {
            min_distance = error;
          }
        }
        if (min_distance <= 0.4) {
          candidate_faces_ortho_horizontal_.push_back(i);
        }
      }
    }
  }
}

void MeshGeneration::computeArtificialVertices(
    pcl::PointCloud<pcl::PointXYZ>::Ptr artificial_vertices,
    std::vector<Eigen::Matrix3d> &shape_directions) {
  Eigen::Vector3f p1;
  Eigen::Vector4f plane_m1, plane_h1, plane_v1;

  for (int i = 0; i < candidate_faces_main_.size(); i++) {
    planeFromIdx(plane_m1, candidate_faces_main_.at(i));

    for (int j = 0; j < candidate_faces_ortho_vertical_.size(); j++) {
      planeFromIdx(plane_v1, candidate_faces_ortho_vertical_.at(j));

      for (int k = 0; k < candidate_faces_ortho_horizontal_.size(); k++) {
        planeFromIdx(plane_h1, candidate_faces_ortho_horizontal_.at(k));

        pcl::threePlanesIntersection(plane_m1, plane_v1, plane_h1, p1);
        artificial_vertices->push_back(pcl::PointXYZ(p1.x(), p1.y(), p1.z()));
      }
    }
  }

  Eigen::Matrix3d shape_direction;
  shape_direction.setZero();
  Eigen::Vector3d n1_zero = mesh_plane_normals_.at(candidate_faces_main_.at(0));
  Eigen::Vector3d n2_zero =
      mesh_plane_normals_.at(candidate_faces_ortho_vertical_.at(0));
  Eigen::Vector3d n3_zero =
      mesh_plane_normals_.at(candidate_faces_ortho_horizontal_.at(0));
  for (int i = 0; i < candidate_faces_main_.size(); i++) {
    Eigen::Vector3d n = mesh_plane_normals_.at(candidate_faces_main_.at(i));
    if (n.dot(n1_zero) > 0) {
      shape_direction.col(0) += n;
    } else {
      shape_direction.col(0) -= n;
    }
  }
  shape_direction.col(0).normalize();

  for (int i = 0; i < candidate_faces_ortho_vertical_.size(); i++) {
    Eigen::Vector3d n =
        mesh_plane_normals_.at(candidate_faces_ortho_vertical_.at(i));
    if (n.dot(n2_zero) > 0) {
      shape_direction.col(1) += n;
    } else {
      shape_direction.col(1) -= n;
    }
  }
  shape_direction.col(1).normalize();

  for (int i = 0; i < candidate_faces_ortho_horizontal_.size(); i++) {
    Eigen::Vector3d n =
        mesh_plane_normals_.at(candidate_faces_ortho_horizontal_.at(i));
    if (n.dot(n3_zero) > 0) {
      shape_direction.col(2) += n;
    } else {
      shape_direction.col(2) -= n;
    }
  }
  shape_direction.col(2).normalize();
  shape_directions.push_back(shape_direction);
}

void MeshGeneration::planeFromIdx(Eigen::Vector4f &result, int idx) {
  Eigen::Vector3d normals = mesh_plane_normals_.at(idx);
  double d = mesh_plane_d_.at(idx);
  result(0) = (float)normals.x();
  result(1) = (float)normals.y();
  result(2) = (float)normals.z();
  result(3) = (float)d;
}

void MeshGeneration::removeDuplicatedPoints(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double eps) {
  // Source:
  // https://stackoverflow.com/questions/34481190/removing-duplicates-of-3d-points-in-a-vector-in-c/34481426
  std::sort(cloud->begin(), cloud->end(),
            [eps](pcl::PointXYZ p1, pcl::PointXYZ p2) -> bool {
              if (std::fabs(p1.x - p2.x) > eps)
                return p1.x > p2.x;
              else if (std::fabs(p1.y - p2.y) > eps)
                return p1.y > p2.y;
              else
                return p1.z > p2.z;
            });
  auto unique_end =
      std::unique(cloud->begin(), cloud->end(),
                  [eps](pcl::PointXYZ p1, pcl::PointXYZ p2) -> bool {
                    if (pcl::euclideanDistance(p1, p2) < eps) {
                      return true;
                    }
                    return false;
                  });
  cloud->erase(unique_end, cloud->end());
}

void MeshGeneration::removeDuplicatedValues(std::vector<double> &vector,
                                            double eps) {
  // Source:
  // https://stackoverflow.com/questions/34481190/removing-duplicates-of-3d-points-in-a-vector-in-c/34481426
  std::sort(vector.begin(), vector.end(),
            [eps](double p1, double p2) -> bool { return p1 > p2; });
  auto unique_end = std::unique(vector.begin(), vector.end(),
                                [eps](double p1, double p2) -> bool {
                                  if (std::fabs(p1 - p2) < eps) {
                                    return true;
                                  }
                                  return false;
                                });
  vector.erase(unique_end, vector.end());
}

bool MeshGeneration::getReconstructionParametersPlanes(
    int idx,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &strong_points_reconstruction,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &weak_points,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &backup_points,
    std::vector<Eigen::Vector3d> &center_estimates,
    std::vector<Eigen::Matrix3d> &direction_estimates,
    std::vector<std::vector<Eigen::VectorXd>> &parameter_estimates) {
  std::vector<int> nn_indices{1};
  std::vector<float> nn_dists{1};

  // Get data
  pcl::PointCloud<pcl::PointXYZ>::Ptr element_points =
      element_corners_vector_.at(idx);
  Eigen::Vector3d element_normal = normal_detected_shapes_vector_.at(idx);

  Eigen::Vector3d rp_temp = robot_positions_.at(idx);
  pcl::PointXYZ robot_position(rp_temp.x(), rp_temp.y(), rp_temp.z());
  pcl::PointXYZ mean_point(0, 0, 0);
  for (int i = 0; i < element_points->size(); i++) {
    mean_point.x += (*element_points)[i].x;
    mean_point.y += (*element_points)[i].y;
    mean_point.z += (*element_points)[i].z;
  }
  mean_point.x /= element_points->size();
  mean_point.y /= element_points->size();
  mean_point.z /= element_points->size();
  Eigen::Vector3d mean_e(mean_point.x, mean_point.y, mean_point.z);
  Eigen::Vector3d shape_robot_vec(mean_point.x - robot_position.x,
                                  mean_point.y - robot_position.y,
                                  mean_point.z - robot_position.z);
  shape_robot_vec.normalize();
  // Normal direction of shape is pointing into free direction
  if (shape_robot_vec.dot(element_normal) < 0.0) {
    ROS_INFO("Error: Normal pointing in the wrong direction!");
    assert(false);
  }

  double shape_d =
      -(mean_point.x * element_normal.x() + mean_point.y * element_normal.y() +
        mean_point.z * element_normal.z());

  // Filter points (remaining vertices -> vertices on axis and free side)
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = 0; i < weak_points->size(); i++) {
    pcl::PointXYZ p = (*weak_points)[i];
    if (p.x * element_normal.x() + p.y * element_normal.y() +
            p.z * element_normal.z() + shape_d >=
        -0.05) {
      filtered_cloud->push_back(p);
    }
  }

  // Corp off confliction points
  // Source: https://pcl.readthedocs.io/en/latest/moment_of_inertia.html
  /*
  pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud(filtered_cloud);
  feature_extractor.compute();
  Eigen::Vector3f major_vector, middle_vector, minor_vector;
  Eigen::Vector3f mass_center;
  feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
  feature_extractor.getMassCenter(mass_center);
  */
  Eigen::Vector3f mass_center;
  mass_center = mean_e.cast<float>() + 0.05 * element_normal.cast<float>();

  Eigen::Matrix3d corrected_dir = shape_directions_.at(idx);
  direction_estimates.push_back(corrected_dir);

  Eigen::Matrix<float, 3, 6> shooting_dir;
  shooting_dir.col(0) = corrected_dir.col(0).cast<float>();
  shooting_dir.col(1) = -corrected_dir.col(0).cast<float>();
  shooting_dir.col(2) = corrected_dir.col(1).cast<float>();
  shooting_dir.col(3) = -corrected_dir.col(1).cast<float>();
  shooting_dir.col(4) = corrected_dir.col(2).cast<float>();
  shooting_dir.col(5) = -corrected_dir.col(2).cast<float>();

  std::cout << shooting_dir << std::endl;
  for (int i = 0; i < 6; i++) {
    pcl::PointCloud<pcl::PointXYZ>::VectorType voxel_centers;
    Eigen::Vector3f direction = shooting_dir.col(i);
    model_octree_->getIntersectedVoxelCenters(mass_center, direction,
                                              voxel_centers);

    if (voxel_centers.size() > 0) {
      pcl::PointXYZ voxel_center = voxel_centers.at(0);

      model_upsampled_kdtree_->nearestKSearch(voxel_center, 1, nn_indices,
                                              nn_dists);
      pcl::PointXYZ boundary_point = (*upsampled_model_)[nn_indices.at(0)];

      double d_boundary = -(direction.x() * boundary_point.x +
                            direction.y() * boundary_point.y +
                            direction.z() * boundary_point.z);

      pcl::PointIndices::Ptr indices_to_remove(new pcl::PointIndices());
      for (int j = 0; j < filtered_cloud->size(); j++) {
        pcl::PointXYZ p = (*filtered_cloud)[j];
        double error = p.x * direction.x() + p.y * direction.y() +
                       p.z * direction.z() + d_boundary;
        if (error > 0.04) {
          indices_to_remove->indices.push_back(j);
        }
      }
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud(filtered_cloud);
      extract.setIndices(indices_to_remove);
      extract.setNegative(true);
      extract.filter(*filtered_cloud);
    }
  }

  // feature_extractor.setInputCloud(filtered_cloud);
  // feature_extractor.compute();
  // feature_extractor.getMassCenter(mass_center);
  // center_estimates.push_back(mass_center.cast<double>());
  /*
  double max_n = -1000;
  double min_n = 1000;
  for (int i = 0; i < filtered_cloud->size(); i++) {
    pcl::PointXYZ cur_p = (*filtered_cloud)[i];
    Eigen::Vector3d cur_e(cur_p.x, cur_p.y, cur_p.z);
    double cur_n = element_normal.dot((cur_e - mean_e));
    if (cur_n > max_n) {
      max_n = cur_n;
    }
    if (cur_n < min_n) {
      min_n = cur_n;
    }
  }
  double middle_n = (max_n + min_n) / 2.0;
  Eigen::Vector3d new_center = mean_e + middle_n * element_normal;
  center_estimates.push_back(new_center);
  mass_center = new_center.cast<float>();
  */
  center_estimates.push_back(mass_center.cast<double>());

  removeDuplicatedPoints(filtered_cloud, 0.003);

  std::string save2 =
      OUTPUT_DIR_ + "resulting_points" + std::to_string(idx) + ".ply";
  pcl::io::savePLYFile(save2, *filtered_cloud);

  // Structure of strong points
  pcl::PointCloud<pcl::PointXYZ>::Ptr alive_strong_points_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  for (int i = 0; i < strong_points_reconstruction->size(); i++) {
    pcl::PointXYZ p_f = (*strong_points_reconstruction)[i];
    Eigen::Vector3d e_f(p_f.x, p_f.y, p_f.z);
    for (int j = 0; j < filtered_cloud->size(); j++) {
      pcl::PointXYZ p_s = (*filtered_cloud)[j];
      Eigen::Vector3d e_s(p_s.x, p_s.y, p_s.z);
      double error = (e_f - e_s).lpNorm<Eigen::Infinity>();
      if (error < 0.05) {
        alive_strong_points_cloud->push_back(p_f);
        break;
      }
    }
  }
  removeDuplicatedPoints(alive_strong_points_cloud);

  std::string save4 =
      OUTPUT_DIR_ + "alive_strong_points_cloud" + std::to_string(idx) + ".ply";
  pcl::io::savePLYFile(save4, *alive_strong_points_cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr alive_element_points(
      new pcl::PointCloud<pcl::PointXYZ>());
  std::vector<Eigen::Vector3d> alive_element_points_normal;
  for (int i = 0; i < element_corners_vector_.size(); i++) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cur_element_points =
        element_corners_vector_.at(i);
    Eigen::Vector3d cur_element_normal = normal_detected_shapes_vector_.at(i);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr element_kdtree =
        element_points_kdtrees_vector_.at(i);
    for (int j = 0; j < filtered_cloud->size(); j++) {
      pcl::PointXYZ p_s = (*filtered_cloud)[j];
      element_kdtree->nearestKSearch(p_s, 1, nn_indices, nn_dists);
      if (std::sqrt(nn_dists[0]) < 0.01) {
        alive_element_points->push_back(p_s);
        alive_element_points_normal.push_back(cur_element_normal);
      }
    }
  }

  ROS_INFO("Nr. Strong Point alive: %d", alive_strong_points_cloud->size());
  ROS_INFO("Nr. Element Points alive:  %d", alive_element_points->size());

  // a1, a2, b1, b2, c1, c2
  // a -> direction of major_vector
  // b -> direction of middle_vector
  // c -> direction of minor_vector
  std::vector<double> a1, a2, b1, b2, c1, c2;

  for (int i = 0; i < alive_strong_points_cloud->size(); i++) {
    pcl::PointXYZ cur_point = (*alive_strong_points_cloud)[i];
    Eigen::Vector3f e_p(cur_point.x, cur_point.y, cur_point.z);
    Eigen::Vector3f center_point = e_p - mass_center;
    double a = center_point.dot(corrected_dir.col(0).cast<float>());
    double b = center_point.dot(corrected_dir.col(1).cast<float>());
    double c = center_point.dot(corrected_dir.col(2).cast<float>());
    if (a > 0) {
      a1.push_back(a);
    } else {
      a2.push_back(a);
    }
    if (b > 0) {
      b1.push_back(b);
    } else {
      b2.push_back(b);
    }
    if (c > 0) {
      c1.push_back(c);
    } else {
      c2.push_back(c);
    }
  }

  bool a1_incomplete = a1.empty();
  bool a2_incomplete = a2.empty();
  bool b1_incomplete = b1.empty();
  bool b2_incomplete = b2.empty();
  bool c1_incomplete = c1.empty();
  bool c2_incomplete = c2.empty();

  for (int i = 0; i < alive_element_points->size(); i++) {
    pcl::PointXYZ cur_point = (*alive_element_points)[i];
    Eigen::Vector3f e_p(cur_point.x, cur_point.y, cur_point.z);
    Eigen::Vector3f center_point = e_p - mass_center;
    double a = center_point.dot(corrected_dir.col(0).cast<float>());
    double b = center_point.dot(corrected_dir.col(1).cast<float>());
    double c = center_point.dot(corrected_dir.col(2).cast<float>());

    Eigen::Vector3d cur_element_normal = alive_element_points_normal.at(i);
    bool ortho_a =
        std::fabs(cur_element_normal.dot(corrected_dir.col(0))) > 0.9;
    bool ortho_b =
        std::fabs(cur_element_normal.dot(corrected_dir.col(1))) > 0.9;
    bool ortho_c =
        std::fabs(cur_element_normal.dot(corrected_dir.col(2))) > 0.9;

    if (ortho_a) {
      if (a > 0 && a1_incomplete) {
        a1.push_back(a);
      } else if (a < 0 && a2_incomplete) {
        a2.push_back(a);
      }
    }

    if (ortho_b) {
      if (b > 0 && b1_incomplete) {
        b1.push_back(b);
      } else if (b < 0 && b2_incomplete) {
        b2.push_back(b);
      }
    }
    if (ortho_c) {
      if (c > 0 && c1_incomplete) {
        c1.push_back(c);
      } else if (c < 0 && c2_incomplete) {
        c2.push_back(c);
      }
    }
  }

  a1_incomplete = a1.empty();
  a2_incomplete = a2.empty();
  b1_incomplete = b1.empty();
  b2_incomplete = b2.empty();
  c1_incomplete = c1.empty();
  c2_incomplete = c2.empty();

  for (int i = 0; i < filtered_cloud->size(); i++) {
    pcl::PointXYZ cur_point = (*filtered_cloud)[i];
    Eigen::Vector3f e_p(cur_point.x, cur_point.y, cur_point.z);
    Eigen::Vector3f center_point = e_p - mass_center;
    double a = center_point.dot(corrected_dir.col(0).cast<float>());
    double b = center_point.dot(corrected_dir.col(1).cast<float>());
    double c = center_point.dot(corrected_dir.col(2).cast<float>());
    if (a > 0 && a1_incomplete) {
      a1.push_back(a);
    } else if (a < 0 && a2_incomplete) {
      a2.push_back(a);
    }

    if (b > 0 && b1_incomplete) {
      b1.push_back(b);
    } else if (b < 0 && b2_incomplete) {
      b2.push_back(b);
    }

    if (c > 0 && c1_incomplete) {
      c1.push_back(c);
    } else if (c < 0 && c2_incomplete) {
      c2.push_back(c);
    }
  }

  removeDuplicatedValues(a1, 0.03);
  removeDuplicatedValues(a2, 0.03);
  removeDuplicatedValues(b1, 0.03);
  removeDuplicatedValues(b2, 0.03);
  removeDuplicatedValues(c1, 0.03);
  removeDuplicatedValues(c2, 0.03);

  Eigen::VectorXd a1_vec =
      Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(a1.data(), a1.size());
  Eigen::VectorXd a2_vec =
      Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(a2.data(), a2.size());
  Eigen::VectorXd b1_vec =
      Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(b1.data(), b1.size());
  Eigen::VectorXd b2_vec =
      Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(b2.data(), b2.size());
  Eigen::VectorXd c1_vec =
      Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(c1.data(), c1.size());
  Eigen::VectorXd c2_vec =
      Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(c2.data(), c2.size());

  std::vector<Eigen::VectorXd> parameters;
  parameters.push_back(a1_vec);
  parameters.push_back(a2_vec);
  parameters.push_back(b1_vec);
  parameters.push_back(b2_vec);
  parameters.push_back(c1_vec);
  parameters.push_back(c2_vec);
  parameter_estimates.push_back(parameters);

  return true;
}

}  // namespace cpt_reconstruction
}  // namespace cad_percept