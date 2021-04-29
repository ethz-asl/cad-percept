#include <cpt_reconstruction/reconstruction_model.h>

namespace cad_percept {
namespace cpt_reconstruction {

Model::Model(std::string filename, Eigen::Matrix4d transformation)
    : meshing_points_(new pcl::PointCloud<pcl::PointXYZ>),
      filename_(filename),
      transformation_(transformation) {}

void Model::preprocess() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr model_points(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PLYReader reader;
  reader.read(filename_, *model_points);
  model_points_ = model_points;

  // Build Search Tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTree(
      new pcl::search::KdTree<pcl::PointXYZ>());
  searchTree->setInputCloud(model_points_);
  searchTree_ = searchTree;
  ROS_INFO("[Done] Build up search tree\n");
}

void Model::queryTree(pcl::PointXYZ p) {
  searchTree_->nearestKSearch(p, 1, nn_indices_, nn_dists_);
}

void Model::addOutlier(pcl::PointXYZ p) { meshing_points_->push_back(p); }

// Source:https://github.com/tttamaki/ICP-test/blob/master/src/icp3_with_normal_iterative_view.cpp
void Model::addNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                       pcl::PointCloud<pcl::Normal>::Ptr normals, int k) {
  pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  searchTree->setInputCloud(cloud);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
  normalEstimator.setInputCloud(cloud);
  normalEstimator.setSearchMethod(searchTree);
  normalEstimator.setKSearch(k);
  normalEstimator.compute(*normals);
}

void Model::applyFilter() {
  ROS_INFO("Size before VoxelGridFiltering: %d\n", meshing_points_->size());
  pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ> octree_filter(
      0.01f);
  octree_filter.setInputCloud(meshing_points_);
  octree_filter.addPointsFromInputCloud();
  pcl::PointCloud<pcl::PointXYZ>::VectorType voxelCentroids;
  octree_filter.getVoxelCentroids(voxelCentroids);
  meshing_points_->clear();
  for (int i = 0; i < voxelCentroids.size(); i++) {
    meshing_points_->push_back(voxelCentroids[i]);
  }

  /*
  Error for too big point clouds
  [pcl::VoxelGrid::applyFilter] Leaf size is too small for the input dataset.
  Integer indices would overflow.[ pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setInputCloud(meshing_points_);
  voxel_grid.setLeafSize(0.01f, 0.01f, 0.01f);
  voxel_grid.filter(*meshing_points_);
  */
  ROS_INFO("Size after VoxelGridFiltering: %d\n", meshing_points_->size());
};

void Model::efficientRANSAC() {
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  this->addNormals(meshing_points_, normals, 15);

  std::vector<Point_with_normal> outliers(meshing_points_->size());
  for (unsigned i = 0; i < meshing_points_->size(); i++) {
    double x = (*meshing_points_)[i].x;
    double y = (*meshing_points_)[i].y;
    double z = (*meshing_points_)[i].z;

    double dx = (*normals)[i].normal_x;
    double dy = (*normals)[i].normal_y;
    double dz = (*normals)[i].normal_z;

    Kernel::Point_3 p_temp(x, y, z);
    Kernel::Vector_3 n_temp(dx, dy, dz);

    std::pair<Kernel::Point_3, Kernel::Vector_3> res(p_temp, n_temp);
    outliers[i] = res;
  }

  Efficient_ransac ransac;
  ransac.set_input(outliers);
  ransac.add_shape_factory<Plane>();
  ransac.add_shape_factory<Cylinder>();

  Efficient_ransac::Parameters parameters;
  parameters.probability = 0.0001;
  parameters.min_points = 150;
  parameters.epsilon = 0.03;
  parameters.cluster_epsilon = 0.1;  // 0.5
  parameters.normal_threshold = 0.90;

  ransac.detect(parameters);

  ROS_INFO("Detected Shapes: %d\n",
           ransac.shapes().end() - ransac.shapes().begin());
  ROS_INFO("Unassigned Points: %d\n", ransac.number_of_unassigned_points());

  pcl::PointIndices::Ptr detected_points(new pcl::PointIndices());
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  std::ofstream file;
  file.open("/home/philipp/Schreibtisch/outliers_ros.xyz", std::ofstream::app);
  Efficient_ransac::Shape_range shapes = ransac.shapes();
  Efficient_ransac::Shape_range::iterator it = shapes.begin();

  while (it != shapes.end()) {
    if (Plane* plane = dynamic_cast<Plane*>(it->get())) {
      const std::vector<std::size_t> idx_assigned_points =
          plane->indices_of_assigned_points();

      Kernel::Plane_3 plane_3 = static_cast<Kernel::Plane_3>(*plane);

      Kernel::Vector_3 ransac_normal_temp = (*plane).plane_normal();
      Eigen::Vector3d ransac_normal(ransac_normal_temp.x(),
                                    ransac_normal_temp.y(),
                                    ransac_normal_temp.z());
      ransac_normal.normalize();

      Eigen::MatrixXd points(3, idx_assigned_points.size());
      for (unsigned i = 0; i < idx_assigned_points.size(); i++) {
        detected_points->indices.push_back(idx_assigned_points.at(i));

        Point_with_normal point_with_normal =
            outliers[idx_assigned_points.at(i)];
        Kernel::Point_3 p = point_with_normal.first;

        // Project points to plane
        Eigen::Vector3d p_orig(p.x(), p.y(), p.z());
        Eigen::Vector3d p_proj =
            p_orig - (p_orig.dot(ransac_normal) + plane_3.d()) * ransac_normal;
        file << p_proj.x() << " " << p_proj.y() << " " << p_proj.z() << "\n";
        points.block<3, 1>(0, i) =
            Eigen::Vector3d(p_proj.x(), p_proj.y(), p_proj.z());
      }
      points_shape_.push_back(points);
      ransac_normals_.push_back(ransac_normal);
      shape_id_.push_back(0);

    } /* else if (Cylinder* cyl = dynamic_cast<Cylinder*>(it->get())) {
       const std::vector<std::size_t> idx_assigned_points =
           cyl->indices_of_assigned_points();
       Eigen::MatrixXd points(3, idx_assigned_points.size());
       Eigen::MatrixXd normals(3, idx_assigned_points.size());
       for (unsigned i = 0; i < idx_assigned_points.size(); i++) {
         detected_points->indices.push_back(idx_assigned_points.at(i));
         Point_with_normal point_with_normal = outliers[idx_assigned_points[i]];
         Kernel::Point_3 p = point_with_normal.first;
         Kernel::Vector_3 n = point_with_normal.second;
         file << p.x() << " " << p.y() << " " << p.z() << "\n";
         points.block<3, 1>(0, i) = Eigen::Vector3d(p.x(), p.y(), p.z());
         normals.block<3, 1>(0, i) = Eigen::Vector3d(n.x(), n.y(), n.z());
       }
       points_shape_.push_back(points);
       normals_shape_.push_back(points);
       shape_id_.push_back(1);
     }*/
    it++;
  }
  file.close();

  extract.setInputCloud(meshing_points_);
  extract.setIndices(detected_points);
  extract.setNegative(true);
  extract.filter(*meshing_points_);
}

// Source:
// https://github.com/apalomer/plane_fitter/blob/master/src/check_planarity.cpp
// and
// https://pointclouds.org/documentation/tutorials/cylinder_segmentation.html
void Model::SACSegmentation() {
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.03);
  seg.setMaxIterations(800);
  // seg.setRadiusLimits(0.05, 0.5);

  std::vector<Eigen::Vector3f> axis_vec;
  for (int i = 0; i < 180; i += 20) {
    float rad = (float)i * (M_PI / 180.0);
    Eigen::Vector3f ax(cos(rad), sin(rad), 0.0f);
    axis_vec.push_back(ax);
  }
  // axis_vec.push_back(Eigen::Vector3f(0, 0, 1));

  for (int c = 0; c < axis_vec.size(); c++) {
    for (int it = 0; it < 2; it++) {
      if (meshing_points_->size() < 2000) {
        break;
      }
      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
          new pcl::PointCloud<pcl::Normal>);
      this->addNormals(meshing_points_, cloud_normals, 100);
      seg.setInputNormals(cloud_normals);
      seg.setInputCloud(meshing_points_);
      seg.setAxis(axis_vec.at(c));
      seg.setEpsAngle(0.175);
      seg.segment(*inliers, *coefficients);

      Eigen::Vector3d plane_normal(coefficients->values[0],
                                   coefficients->values[1],
                                   coefficients->values[2]);
      plane_normal = plane_normal.normalized();

      // Project points to plane
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(
          new pcl::PointCloud<pcl::PointXYZ>);
      int nr_inliers = inliers->indices.size();
      if (nr_inliers <= 50) {
        break;
      }
      for (unsigned i = 0; i < nr_inliers; i++) {
        pcl::PointXYZ p = meshing_points_->points[inliers->indices[i]];
        Eigen::Vector3d p_orig(p.x, p.y, p.z);
        Eigen::Vector3d p_proj =
            p_orig -
            (p_orig.dot(plane_normal) + coefficients->values[3]) * plane_normal;
        cloud_projected->push_back(
            pcl::PointXYZ(p_orig.x(), p_orig.y(), p_orig.z()));
      }

      // Source: https://pcl.readthedocs.io/en/latest/cluster_extraction.html
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
          new pcl::search::KdTree<pcl::PointXYZ>);
      tree->setInputCloud(cloud_projected);
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec_clustering;
      ec_clustering.setClusterTolerance(0.25);
      ec_clustering.setMinClusterSize(50);
      ec_clustering.setMaxClusterSize(500000);
      ec_clustering.setSearchMethod(tree);
      ec_clustering.setInputCloud(cloud_projected);

      std::vector<pcl::PointIndices> clusters;
      ec_clustering.extract(clusters);

      for (int i = 0; i < clusters.size(); i++) {
        std::vector<int> indices = clusters.at(i).indices;
        int indices_size = indices.size();
        if (indices_size > 50) {
          Eigen::MatrixXd points(3, indices_size);
          for (int j = 0; j < indices_size; j++) {
            pcl::PointXYZ p_c = (*cloud_projected)[indices.at(j)];
            points.block<3, 1>(0, j) = Eigen::Vector3d(p_c.x, p_c.y, p_c.z);
          }
          points_shape_.push_back(points);
          ransac_normals_.push_back(plane_normal);
          shape_id_.push_back(0);
        }
      }
      extract.setInputCloud(meshing_points_);
      extract.setIndices(inliers);
      extract.setNegative(true);
      extract.filter(*meshing_points_);
    }
  }
}

std::vector<Eigen::MatrixXd>* Model::getPointShapes() { return &points_shape_; }

std::vector<Eigen::Vector3d>* Model::getRansacNormals() {
  return &ransac_normals_;
}

std::vector<int>* Model::getShapeIDs() { return &shape_id_; }

int Model::getOutlierCount() { return meshing_points_->size(); }

float Model::getMinDistance() { return nn_dists_[0]; }

void Model::clearRansacShapes() {
  points_shape_.clear();
  ransac_normals_.clear();
  shape_id_.clear();
}

void Model::clearBuffer() { meshing_points_->clear(); }

}  // namespace cpt_reconstruction
}  // namespace cad_percept