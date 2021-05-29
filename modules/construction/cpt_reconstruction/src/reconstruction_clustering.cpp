#include <cpt_reconstruction/reconstruction_clustering.h>

namespace cad_percept {
namespace cpt_reconstruction {
Clustering::Clustering(ros::NodeHandle nodeHandle1, ros::NodeHandle nodeHandle2)
    : nodeHandle1_(nodeHandle1),
      nodeHandle2_(nodeHandle2),
      counter_(0),
      received_shapes_(0) {
  // preprocessBuildingModel();
  subscriber_ =
      nodeHandle1_.subscribe("ransac_shape", 1000, &Clustering::messageCallback, this);
  publisher_ =
      nodeHandle2_.advertise<::cpt_reconstruction::clusters>("clusters", 1000);
  ros::spin();
}

// Source:
// https://pointclouds.org/documentation/tutorials/greedy_projection.html
// https://pointclouds.org/documentation/tutorials/resampling.html
void Clustering::messageCallback(const ::cpt_reconstruction::shape &msg) {
  ROS_INFO("[Mesh Generation] Id: %d with size: %d", msg.id,
           msg.points_msg.size());

  if (msg.id == 0) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr points_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<geometry_msgs::Vector3> points = msg.points_msg;
    geometry_msgs::Vector3 ransac_normal_temp = msg.ransac_normal;
    Eigen::Vector3d ransac_normal(ransac_normal_temp.x, ransac_normal_temp.y,
                                  ransac_normal_temp.z);
    geometry_msgs::Vector3 rp_temp = msg.robot_position;
    Eigen::Vector3d robot_position(rp_temp.x, rp_temp.y, rp_temp.z);

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
      robot_positions_.push_back(robot_position);

      if (counter_ >= 50 && (counter_ % 50 == 0)) {
        ROS_INFO("Size before fuseing: %d \n", clouds_.size());
        this->fusePlanes();
        ROS_INFO("Size after fuseing: %d \n", clouds_.size());
      }

      if (counter_ >= 200 && (counter_ % 200 == 0)) {
        ROS_INFO("Size before fuseing: %d \n", clouds_.size());
        this->fusePlanes();
        ROS_INFO(
            "Size after fuseing and before removing conflicting clusters:: %d "
            "\n",
            clouds_.size());
        this->removeSingleDetections();
        this->removeConflictingClusters();
        ROS_INFO("Size after removing conflicting clusters: %d \n",
                 clouds_.size());
      }

      // TODO - Change ~200
      if (counter_ >= 200 && (counter_ % 200 == 0)) {
        std::vector<sensor_msgs::PointCloud2> clusters_vec;
        std::vector<geometry_msgs::Vector3> robot_positions_vec;
        std::vector<geometry_msgs::Vector3> ransac_normal_vec;
        std::vector<double> radius_vec;
        std::vector<geometry_msgs::Vector3> axis_vec;
        std::vector<int> id_vec;

        for (unsigned i = 0; i < clouds_.size(); i++) {
          if (fusing_count_.at(i) > 0) {
            pcl::PCLPointCloud2 temp_pcl;
            pcl::toPCLPointCloud2(*(clouds_.at(i)), temp_pcl);

            sensor_msgs::PointCloud2 temp_ros;
            pcl_conversions::moveFromPCL(temp_pcl, temp_ros);
            clusters_vec.push_back(temp_ros);

            geometry_msgs::Vector3 temp_robot_pos;
            temp_robot_pos.x = robot_positions_.at(i).x();
            temp_robot_pos.y = robot_positions_.at(i).y();
            temp_robot_pos.z = robot_positions_.at(i).z();
            robot_positions_vec.push_back(temp_robot_pos);

            geometry_msgs::Vector3 ransac_normal_temp;
            ransac_normal_temp.x = ransac_normals_.at(i).x();
            ransac_normal_temp.y = ransac_normals_.at(i).y();
            ransac_normal_temp.z = ransac_normals_.at(i).z();
            ransac_normal_vec.push_back(ransac_normal_temp);

            geometry_msgs::Vector3 axis_temp;
            axis_temp.x = 0;
            axis_temp.y = 0;
            axis_temp.z = 0;

            axis_vec.push_back(axis_temp);
            radius_vec.push_back(0.0);
            id_vec.push_back(0);
          }
        }
        ::cpt_reconstruction::clusters cluster_msg;
        cluster_msg.clouds = clusters_vec;
        cluster_msg.robot_positions = robot_positions_vec;
        cluster_msg.ransac_normal = ransac_normal_vec;
        cluster_msg.axis = axis_vec;
        cluster_msg.radius = radius_vec;
        cluster_msg.id = id_vec;

        ROS_INFO("Published Clusters");
        publisher_.publish(cluster_msg);
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

      ROS_INFO("[Mesh Generation] Counter: %d", counter_);
      counter_++;
    }
  }
}

bool Clustering::checkValidPlane(
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
      ROS_INFO("Valid plane %d with l2 = %f\n", received_shapes_, middle_value);
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

void Clustering::fusePlanes() {
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

      double dot_prod = ransac_normals_[i].dot(ransac_normals_[j]);
      if (std::abs(dot_prod) > 0.99) {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_i = kd_trees_[i];
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_j = clouds_[j];
        int matches = 0;
        for (int k = 0; k < cloud_j->size(); k++) {
          pcl::PointXYZ p = (*cloud_j)[k];
          tree_i->nearestKSearch(p, 1, nn_indices, nn_dists);
          if (std::sqrt(nn_dists[0]) < 0.05) {
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
  std::vector<Eigen::Vector3d> robot_positions;

  for (int i = 0; i < fusing.size(); i++) {
    std::vector<int> fusing_vec = fusing.at(i);
    pcl::PointCloud<pcl::PointXYZ>::Ptr fused_point_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Vector3d new_robot_position(0.0, 0.0, 0.0);
    int count_fused_shapes = 0;
    for (int j = 0; j < fusing_vec.size(); j++) {
      (*fused_point_cloud) += (*(clouds_[fusing_vec.at(j)]));
      count_fused_shapes += fusing_count_.at(fusing_vec.at(j));
      // TODO - Maybe compute weighted average?
      new_robot_position += robot_positions_.at(fusing_vec.at(j));
    }
    new_robot_position /= fusing_vec.size();

    if (fused_point_cloud->size() > 10) {
      pcl::VoxelGrid<pcl::PointXYZ> sor;
      sor.setInputCloud(fused_point_cloud);
      sor.setLeafSize(0.01f, 0.01f, 0.01f);
      sor.filter(*fused_point_cloud);

      //
      // Recompute ransac normal and project/filter(??) points
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
      pcl::SACSegmentation<pcl::PointXYZ> seg;

      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setDistanceThreshold(0.05);
      seg.setMaxIterations(1000);
      seg.setInputCloud(fused_point_cloud);
      seg.segment(*inliers, *coefficients);

      /*
      int num_points = fused_point_cloud->size();
      Eigen::MatrixXd A(num_points, 4);
      Eigen::VectorXd B(num_points);
      for (int i = 0; i < num_points; i++){
        pcl::PointXYZ p = (*fused_point_cloud)[i];
        A.block<1, 4>(i, 0) = Eigen::Vector4d(p.x, p.y, p.z, 1.0);
        B(i) = 0;
      }
      Eigen::Vector4d plane_coeff = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);
      Eigen::Vector3d plane_normal(plane_coeff(0),plane_coeff(1),
                                   plane_coeff(2));
      plane_normal.normalize();
      double mean_x = A.col(0).mean();
      double mean_y = A.col(1).mean();
      double mean_z = A.col(2).mean();
      double plane_d = -(mean_x * plane_normal.x() + mean_y * plane_normal.y() + mean_z * plane_normal.z());
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
      coefficients->values.push_back(plane_normal.x());
      coefficients->values.push_back(plane_normal.y());
      coefficients->values.push_back(plane_normal.z());
      coefficients->values.push_back(plane_d);
      */

      /*
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud(fused_point_cloud);
      extract.setIndices(inliers);
      extract.setNegative(false);
      extract.filter(*fused_point_cloud);
      */
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
      robot_positions.push_back(new_robot_position);
    }
  }
  clouds_.clear();
  ransac_normals_.clear();
  kd_trees_.clear();
  fusing_count_.clear();
  robot_positions_.clear();

  clouds_ = fused_clouds;
  ransac_normals_ = fused_ransac_normals;
  kd_trees_ = fused_kd_trees;
  fusing_count_ = fusing_count;
  robot_positions_ = robot_positions;
}

void Clustering::removeSingleDetections() {
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
    robot_positions_.erase(robot_positions_.begin() + idx - idx_corr);
    idx_corr++;
  }
}

void Clustering::removeConflictingClusters() {
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
        if (std::sqrt(nn_dists[0]) < 0.03) {
          matches++;
        }
      }
    }
    double coverage = ((double)matches) / ((double)cloud_i->size());
    if (coverage >= 0.4) {
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
    robot_positions_.erase(robot_positions_.begin() + idx);
  }
}

void Clustering::fit3DPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
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
  detected_shapes_points_.push_back(corners_no_i);
  Eigen::Vector3d plane_normal(coefficients->values[0], coefficients->values[1],
                               coefficients->values[2]);
  plane_normal.normalize();
  detected_shapes_params_.push_back(
      Eigen::Vector4d(plane_normal.x(), plane_normal.y(), plane_normal.z(),
                      coefficients->values[3]));

  // Compute Convex? or Concave Hull
  pcl::ConvexHull<pcl::PointXYZI> chull;
  chull.setInputCloud(corners);
  // chull.setAlpha (0.1);
  chull.reconstruct(mesh);
}

void Clustering::combineMeshes(const pcl::PolygonMesh &mesh,
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

}  // namespace cpt_reconstruction
}  // namespace cad_percept