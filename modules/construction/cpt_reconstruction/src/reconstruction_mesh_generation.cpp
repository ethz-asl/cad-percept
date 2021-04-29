#include <cpt_reconstruction/reconstruction_mesh_generation.h>

namespace cad_percept {
namespace cpt_reconstruction {
MeshGeneration::MeshGeneration(ros::NodeHandle nodeHandle)
    : nodeHandle_(nodeHandle), counter_(0), received_shapes_(0) {
  subscriber_ = nodeHandle_.subscribe("ransac_shape", 1000,
                                      &MeshGeneration::messageCallback, this);
  ros::spin();
}

// Source:
// https://pointclouds.org/documentation/tutorials/greedy_projection.html
// https://pointclouds.org/documentation/tutorials/resampling.html
void MeshGeneration::messageCallback(const ::cpt_reconstruction::shape &msg) {
  ROS_INFO("[Mesh Generation] Id: %d with size: %d", msg.id,
           msg.points_msg.size());

  pcl::PointCloud<pcl::PointXYZ>::Ptr points_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<geometry_msgs::Vector3> points = msg.points_msg;
  geometry_msgs::Vector3 ransac_normal_temp = msg.ransac_normal;
  Eigen::Vector3d ransac_normal(ransac_normal_temp.x, ransac_normal_temp.y,
                                ransac_normal_temp.z);
  for (unsigned i = 0; i < points.size(); i++) {
    geometry_msgs::Vector3 p = points.at(i);
    points_cloud->push_back(pcl::PointXYZ(p.x, p.y, p.z));
  }

  bool valid_plane = checkValidPlane(points_cloud, 1000, 50);

  if (valid_plane) {
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(points_cloud);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*points_cloud);

    clouds_.push_back(points_cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree(
        new pcl::search::KdTree<pcl::PointXYZ>());
    kd_tree->setInputCloud(points_cloud);
    kd_trees_.push_back(kd_tree);
    fusing_count_.push_back(1);
    ransac_normals_.push_back(ransac_normal);

    if (counter_ >= 50 && (counter_ % 50 == 0)) {
      ROS_INFO("Size before fuseing: %d \n", clouds_.size());
      this->fusePlanes();
      ROS_INFO("Size after fuseing: %d \n", clouds_.size());
    }

    if (counter_ >= 200 && (counter_ % 200 == 0)) {
      ROS_INFO("Size before fuseing: %d \n", clouds_.size());
      this->fusePlanes();
      ROS_INFO(
              "Size after fuseing and before removing conflicting clusters:: %d \n",
              clouds_.size());
      // this->removeSingleDetections();
      this->removeConflictingClusters();
      ROS_INFO("Size after removing conflicting clusters: %d \n",
               clouds_.size());
    }

    if (counter_ >= 200 && (counter_ % 200 == 0)) {
      pcl::PolygonMesh mesh_all;
      for (unsigned i = 0; i < clouds_.size(); i++) {
        if (fusing_count_.at(i) > 0) {
          pcl::PolygonMesh mesh;
          this->fit3DPlane(clouds_[i], mesh);
          this->combineMeshes(mesh, mesh_all);

          std::string result = "/home/philipp/Schreibtisch/Meshes/points_" +
                               std::to_string(i) + ".ply";
          pcl::io::savePLYFile(result, *clouds_[i]);

          std::string result2 = "/home/philipp/Schreibtisch/Meshes/mesh_" +
                                std::to_string(i) + ".ply";
          pcl::io::savePLYFile(result2, mesh);
        }
      }
      pcl::io::savePLYFile("/home/philipp/Schreibtisch/Meshes/mesh_all.ply",
                           mesh_all);
    }
    /*
    if (msg.id == 0){
    } else if (msg.id == 0){
    } else {
      ROS_INFO("Unknown shape\n");
    }
    */
    ROS_INFO("[Mesh Generation] Counter: %d", counter_);
    counter_++;
  }
}

bool MeshGeneration::checkValidPlane(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int valid_size,
    int min_size) {
  bool is_valid = false;

  if (cloud->size() >= valid_size) {
    is_valid = true;
    std::string result = "/home/philipp/Schreibtisch/Shapes/valid_shape_" +
                         std::to_string(counter_) + "_plane.ply";
    pcl::io::savePLYFile(result, *cloud);
    ROS_INFO("Valid plane %d\n", received_shapes_);
  } else if (cloud->size() >= min_size) {
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud);
    feature_extractor.compute();
    float major_value, middle_value, minor_value;
    feature_extractor.getEigenValues(major_value, middle_value, minor_value);

    // assumption: w x .10 plane as minimum size
    if (middle_value > 0.00015) {
      is_valid = true;
      ROS_INFO("Valid plane %d with l2 = %f \n", received_shapes_,
               middle_value);
      std::string result = "/home/philipp/Schreibtisch/Shapes/valid_shape_" +
                           std::to_string(received_shapes_) + "_plane.ply";
      pcl::io::savePLYFile(result, *cloud);
    } else {
      ROS_INFO("Invalid plane %d with l2 = %f \n", received_shapes_,
               middle_value);
      std::string result = "/home/philipp/Schreibtisch/Shapes/invalid_shape_" +
                           std::to_string(received_shapes_) + "_plane.ply";
      pcl::io::savePLYFile(result, *cloud);
    }
  }
  received_shapes_++;
  return is_valid;
}

void MeshGeneration::fusePlanes() {
  std::vector<int> blocked_idx;
  std::vector<int> nn_indices{1};
  std::vector<float> nn_dists{1};
  std::vector<std::vector<int>> fusing;
  for (int i = 0; i < clouds_.size(); i++) {
    if (std::find(blocked_idx.begin(), blocked_idx.end(), i) !=
        blocked_idx.end()) {
      continue;
    }
    std::vector<int> fusing_temp;
    fusing_temp.push_back(i);
    for (int j = i + 1; j < clouds_.size(); j++) {
      if (std::find(blocked_idx.begin(), blocked_idx.end(), j) !=
          blocked_idx.end()) {
        continue;
      }
      double diff1 =
          (ransac_normals_[i] - ransac_normals_[j]).lpNorm<Eigen::Infinity>();
      double diff2 =
          (ransac_normals_[i] + ransac_normals_[j]).lpNorm<Eigen::Infinity>();
      if (diff1 < 0.05 || diff2 < 0.05) {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_i = kd_trees_[i];
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_j = clouds_[j];
        int matches = 0;
        for (int k = 0; k < cloud_j->size(); k++) {
          pcl::PointXYZ p = (*cloud_j)[k];
          tree_i->nearestKSearch(p, 1, nn_indices, nn_dists);
          if (nn_dists[0] < 0.01) {
            matches++;
          }
        }
        double coverage = ((double)matches) / ((double)cloud_j->size());
        if (coverage >= 0.01) {
          blocked_idx.push_back(j);
          fusing_temp.push_back(j);
        }
      }
    }
    fusing.push_back(fusing_temp);
  }
  std::vector<pcl::search::KdTree<pcl::PointXYZ>::Ptr> fused_kd_trees;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> fused_clouds;
  std::vector<Eigen::Vector3d> fused_ransac_normals;
  std::vector<int> fusing_count;

  for (int i = 0; i < fusing.size(); i++) {
    std::vector<int> fusing_vec = fusing.at(i);
    pcl::PointCloud<pcl::PointXYZ>::Ptr fused_point_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    int count_fused_shapes = 0;
    for (int j = 0; j < fusing_vec.size(); j++) {
      (*fused_point_cloud) += (*(clouds_[fusing_vec.at(j)]));
      count_fused_shapes += fusing_count_.at(fusing_vec.at(j));
    }
    if (fused_point_cloud->size() > 10) {
      pcl::VoxelGrid<pcl::PointXYZ> sor;
      sor.setInputCloud(fused_point_cloud);
      sor.setLeafSize(0.01f, 0.01f, 0.01f);
      sor.filter(*fused_point_cloud);

      //
      // Recompute ransac normal and filter points
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      pcl::ExtractIndices<pcl::PointXYZ> extract;

      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setDistanceThreshold(0.03);
      seg.setMaxIterations(1000);
      seg.setInputCloud(fused_point_cloud);
      seg.segment(*inliers, *coefficients);

      extract.setInputCloud(fused_point_cloud);
      extract.setIndices(inliers);
      extract.setNegative(false);
      extract.filter(*fused_point_cloud);

      Eigen::Vector3d plane_normal(coefficients->values[0],
                                   coefficients->values[1],
                                   coefficients->values[2]);
      plane_normal.normalize();

      pcl::ProjectInliers<pcl::PointXYZ> proj;
      proj.setModelType(pcl::SACMODEL_PLANE);
      proj.setInputCloud(fused_point_cloud);
      proj.setModelCoefficients(coefficients);
      proj.filter(*fused_point_cloud);

      pcl::search::KdTree<pcl::PointXYZ>::Ptr fused_kd_tree(
          new pcl::search::KdTree<pcl::PointXYZ>());
      fused_kd_tree->setInputCloud(fused_point_cloud);
      fused_clouds.push_back(fused_point_cloud);
      fused_kd_trees.push_back(fused_kd_tree);
      fused_ransac_normals.push_back(plane_normal);
      fusing_count.push_back(count_fused_shapes);
    }
  }
  clouds_.clear();
  ransac_normals_.clear();
  kd_trees_.clear();
  fusing_count_.clear();
  clouds_ = fused_clouds;
  ransac_normals_ = fused_ransac_normals;
  kd_trees_ = fused_kd_trees;
  fusing_count_ = fusing_count;
}

void MeshGeneration::removeSingleDetections() {
  std::vector<int> remove_idx;
  for (int i = 0; i < fusing_count_.size(); i++) {
    if (fusing_count_.at(i) <= 1) {
      remove_idx.push_back(i);
    }
  }
  int idx_corr = 0;
  for (unsigned r = 0; r < remove_idx.size(); r++) {
    int idx = remove_idx.at(r);
    clouds_.erase(clouds_.begin() + idx - idx_corr);
    kd_trees_.erase(kd_trees_.begin() + idx - idx_corr);
    ransac_normals_.erase(ransac_normals_.begin() + idx - idx_corr);
    fusing_count_.erase(fusing_count_.begin() + idx - idx_corr);
    idx_corr++;
  }
}

void MeshGeneration::removeConflictingClusters() {
  std::vector<int> remove_idx;
  std::vector<int> nn_indices{1};
  std::vector<float> nn_dists{1};
  std::vector<int> blocked_idx;
  for (int i = clouds_.size() - 1; i >= 0; i--) {
    if (fusing_count_.at(i) > 10) {
      continue;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_i = clouds_[i];
    int matches = 0;
    for (int j = 0; j < clouds_.size(); j++) {
      if ((i == j) || (std::find(blocked_idx.begin(), blocked_idx.end(), j) !=
                       blocked_idx.end())) {
        continue;
      }
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_j = kd_trees_[j];
      for (int k = 0; k < cloud_i->size(); k++) {
        pcl::PointXYZ p = (*cloud_i)[k];
        tree_j->nearestKSearch(p, 1, nn_indices, nn_dists);
        if (nn_dists[0] < 0.005) {
          matches++;
        }
      }
    }
    double coverage = ((double)matches) / ((double)cloud_i->size());
    if (coverage >= 0.85) {
      remove_idx.push_back(i);
      blocked_idx.push_back(i);
    }
  }
  // Descending order!!
  // no idx_corr needed!
  for (unsigned r = 0; r < remove_idx.size(); r++) {
    int idx = remove_idx.at(r);
    clouds_.erase(clouds_.begin() + idx);
    kd_trees_.erase(kd_trees_.begin() + idx);
    ransac_normals_.erase(ransac_normals_.begin() + idx);
    fusing_count_.erase(fusing_count_.begin() + idx);
  }
}

void MeshGeneration::fit3DPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                pcl::PolygonMesh &mesh) {
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

  int i = 0, nr_points = (int)cloud_copy->points.size();
  while (cloud_copy->points.size() > 0.05 * nr_points) {
    seg.setInputCloud(cloud_copy);
    seg.segment(*inliers, *coefficients);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(
        new pcl::PointCloud<pcl::PointXYZ>);
    extract.setInputCloud(cloud_copy);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_temp);
    *cloud_new += *cloud_temp;

    extract.setInputCloud(cloud_copy);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_copy);
  }

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
  // Project Points to plane

  // Get linear boundary

  // Extrude element

  // Compute corner_points

  // Compute Convex? or Concave Hull
  pcl::ConvexHull<pcl::PointXYZI> chull;
  chull.setInputCloud(corners);
  // chull.setAlpha (0.1);
  chull.reconstruct(mesh);
}

void MeshGeneration::combineMeshes(const pcl::PolygonMesh &mesh,
                                   pcl::PolygonMesh &mesh_all) {
  // pcl::PolygonMesh::concatenate(mesh_all, mesh); ???
  // Source:
  // https://github.com/PointCloudLibrary/pcl/blob/master/common/include/pcl/PolygonMesh.h
  mesh_all.header.stamp = std::max(mesh_all.header.stamp, mesh.header.stamp);

  const auto point_offset = mesh_all.cloud.width * mesh_all.cloud.height;

  pcl::PCLPointCloud2 new_cloud;
  pcl::concatenatePointCloud(mesh_all.cloud, mesh.cloud, new_cloud);
  mesh_all.cloud = new_cloud;

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

bool MeshGeneration::integrateInBuildingModel{
  pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFilePLY("/home/philipp/Schreibtisch/data/CLA_MissingParts_1.ply", mesh);

  std::vector<::pcl::Vertices> faces = mesh.polygons;
  pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(mesh.cloud, *points);

  //std::cout << "Number faces: " << faces.size() << std::endl;

  std::vector<int> blocking_idx;
  std::vector<std::pair<int, int>> quads;

  //Group elements sharing the a half edge and have the same surface normal
  for (int i = 0; i < faces.size(); i++){
    if (std::find(blocking_idx.begin(), blocking_idx.end(), i) !=
        blocking_idx.end()) {
      continue;
    }
    std::vector<uint32_t> vertices_1 = faces[i].vertices;

    pcl::PointXYZ p1_1 = (*points)[vertices_1.at(0)];
    pcl::PointXYZ p1_2 = (*points)[vertices_1.at(1)];
    pcl::PointXYZ p1_3 = (*points)[vertices_1.at(2)];
    Eigen::Vector3d v1_1(p1_1.x, p1_1.y, p1_1.z);
    Eigen::Vector3d v1_2(p1_2.x, p1_2.y, p1_2.z);
    Eigen::Vector3d v1_3(p1_3.x, p1_3.y, p1_3.z);
    double l1_1 = (v1_1 - v1_2).norm();
    double l1_2 = (v1_2 - v1_3).norm();
    double l1_3 = (v1_3 - v1_1).norm();
    double l1_max = (l1_1 > l1_2) ? l1_1 : l1_2;
    l1_max = (l1_3 > l1_max) ? l1_3 : l1_max;

    for (int j = i + 1; j < faces.size(); j++){
      if (std::find(blocking_idx.begin(), blocking_idx.end(), j) !=
          blocking_idx.end()) {
        continue;
      }
      std::vector<uint32_t> vertices_2 = faces[j].vertices;
      int v_matches = 0;

      // Check if two points matches exactly
      //TODO: Assumption that vertices do not exist twice!
      for (int n = 0; n < 3; n++){
        for (int m = 0; m < 3; m++){
          if (vertices_1.at(n) == vertices_2.at(m)){
            v_matches++;
            break;
          }
        }
      }

      if(v_matches == 2){
        pcl::PointXYZ p2_1 = (*points)[vertices_2.at(0)];
        pcl::PointXYZ p2_2 = (*points)[vertices_2.at(1)];
        pcl::PointXYZ p2_3 = (*points)[vertices_2.at(2)];
        Eigen::Vector3d v2_1(p2_1.x, p2_1.y, p2_1.z);
        Eigen::Vector3d v2_2(p2_2.x, p2_2.y, p2_2.z);
        Eigen::Vector3d v2_3(p2_3.x, p2_3.y, p2_3.z);
        double l2_1 = (v2_1 - v2_2).norm();
        double l2_2 = (v2_2 - v2_3).norm();
        double l2_3 = (v2_3 - v2_1).norm();
        double l2_max = (l2_1 > l2_2) ? l2_1 : l2_2;
        l2_max = (l2_3 > l2_max) ? l2_3 : l2_max;

        //Matching half edge
        if(std::abs(l1_max - l2_max) < 10e-15){
          // Check if normal is the same
          Eigen::Vector3d n1 = (v1_1- v1_2).cross(v1_3 - v1_2);
          Eigen::Vector3d n2 = (v2_1- v2_2).cross(v2_3 - v2_2);
          n1.normalize();
          n2.normalize();

          //Check for same normal
          if ( (n1 - n2).lpNorm<Eigen::Infinity>() < 10e-15 || (n1 + n2).lpNorm<Eigen::Infinity>() < 10e-15){
            //Match found
            blocking_idx.push_back(i);
            blocking_idx.push_back(j);
            quads.push_back(std::make_pair(i, j));
            break;
          }
        }
      }
    }
  }

  //std::cout << "Size of pairs: " << quads.size() << std::endl;
  pcl::PolygonMesh mesh_all;
  for (int i = 0; i < quads.size(); i++){
    std::pair<int,int> p = quads.at(i);

    pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<uint32_t> vertices_1 = faces[p.first].vertices;
    std::vector<uint32_t> vertices_2 = faces[p.second].vertices;

    //Start at a correct edge!!
    pcl::PointXYZ p1 = (*points)[vertices_1.at(0)];
    pcl::PointXYZ p2 = (*points)[vertices_1.at(1)];
    pcl::PointXYZ p3 = (*points)[vertices_1.at(2)];
    Eigen::Vector3d v1(p1.x, p1.y, p1.z);
    Eigen::Vector3d v2(p2.x, p2.y, p2.z);
    Eigen::Vector3d v3(p3.x, p3.y, p3.z);
    double l1 = (v1 - v2).norm();
    double l2 = (v2 - v3).norm();
    double l3 = (v3 - v1).norm();

    int start_point;
    if (l1 > l2 && l1 > l3){
      start_point = 1;
    } else if (l2 > l1 && l2 > l3){
      start_point = 2;
    } else if (l3 > l1 && l3 > l1){
      start_point = 3;
    } else{
      return false;
    }

    for (int j = 0; j < 3; j++){
      int idx = vertices_1.at(j);
      plane->push_back((*points)[idx]);
    }

    for (int j = 0; j < 3; j++){
      int idx = vertices_2.at(j);
      if (std::find(vertices_1.begin(), vertices_1.end(), idx) == vertices_1.end()) {
        plane->push_back((*points)[idx]);
      }
    }

    pcl::PolygonMesh mesh;
    pcl::PCLPointCloud2 temp_cloud;
    pcl::toPCLPointCloud2(*plane, temp_cloud);
    std::vector<pcl::Vertices> polygons_new;
    pcl::Vertices vertices_new;

    if (start_point == 1){
      vertices_new.vertices.push_back(0);
      vertices_new.vertices.push_back(3);
      vertices_new.vertices.push_back(1);
      vertices_new.vertices.push_back(2);
    } else if (start_point == 2){
      vertices_new.vertices.push_back(0);
      vertices_new.vertices.push_back(1);
      vertices_new.vertices.push_back(3);
      vertices_new.vertices.push_back(2);
    } else if (start_point == 3){
      vertices_new.vertices.push_back(0);
      vertices_new.vertices.push_back(1);
      vertices_new.vertices.push_back(2);
      vertices_new.vertices.push_back(3);
    }
    combineMeshes(mesh, mesh_all);

  }
  pcl::io::savePLYFile("/home/philipp/Schreibtisch/mesh_all.ply",mesh_all);
  return true;
};

}  // namespace cpt_reconstruction
}  // namespace cad_percept