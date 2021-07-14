#include <cpt_reconstruction/reconstruction_model.h>

namespace cad_percept {
namespace cpt_reconstruction {

Model::Model(const ros::NodeHandle &nodeHandle)
    : meshing_points_(new pcl::PointCloud<pcl::PointXYZ>) {
  nodeHandle.getParam("UpsampledBuildingModelFile",
                      UPSAMPLED_BUILDING_MODEL_PATH_);
  nodeHandle.getParam("UseFilter", USE_FILTER_);
  nodeHandle.getParam("OctreeFilterResolution", OCTREE_FILTER_RESOLUTION_);
  nodeHandle.getParam("RANSACProbability", RANSAC_PROBABILITY_);
  nodeHandle.getParam("RANSACMinPoints", RANSAC_MIN_POINTS_);
  nodeHandle.getParam("RANSACEpsilon", RANSAC_EPSILON_);
  nodeHandle.getParam("RANSACClusterEpsilon", RANSAC_CLUSTER_EPSILON_);
  nodeHandle.getParam("RANSACNormalThreshold", RANSAC_NORMAL_THRESHOLD_);
}

void Model::preprocess() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr model_points(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PLYReader reader;
  reader.read(UPSAMPLED_BUILDING_MODEL_PATH_, *model_points);
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
  if (USE_FILTER_) {
    pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ> octree_filter(
        OCTREE_FILTER_RESOLUTION_);
    octree_filter.setInputCloud(meshing_points_);
    octree_filter.addPointsFromInputCloud();
    pcl::PointCloud<pcl::PointXYZ>::VectorType voxelCentroids;
    octree_filter.getVoxelCentroids(voxelCentroids);
    meshing_points_->clear();
    for (int i = 0; i < voxelCentroids.size(); i++) {
      meshing_points_->push_back(voxelCentroids[i]);
    }
  }
}

void Model::efficientRANSAC() {
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  this->addNormals(meshing_points_, normals, 20);

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
  parameters.probability = RANSAC_PROBABILITY_;
  parameters.min_points = RANSAC_MIN_POINTS_;
  parameters.epsilon = RANSAC_EPSILON_;
  parameters.cluster_epsilon = RANSAC_CLUSTER_EPSILON_;
  parameters.normal_threshold = RANSAC_NORMAL_THRESHOLD_;

  ransac.detect(parameters);

  ROS_INFO("Detected Shapes: %d\n",
           ransac.shapes().end() - ransac.shapes().begin());
  ROS_INFO("Unassigned Points: %d\n", ransac.number_of_unassigned_points());

  pcl::PointIndices::Ptr detected_points(new pcl::PointIndices());
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  Efficient_ransac::Shape_range shapes = ransac.shapes();
  Efficient_ransac::Shape_range::iterator it = shapes.begin();

  while (it != shapes.end()) {
    if (Plane *plane = dynamic_cast<Plane *>(it->get())) {
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
        points.block<3, 1>(0, i) =
            Eigen::Vector3d(p_proj.x(), p_proj.y(), p_proj.z());
      }
      points_shape_.push_back(points);
      ransac_normals_.push_back(ransac_normal);
      shape_id_.push_back(0);
      axis_.push_back(Eigen::Vector3d::Zero());
      radius_.push_back(0.0);

    } else if (Cylinder *cyl = dynamic_cast<Cylinder *>(it->get())) {
      const std::vector<std::size_t> idx_assigned_points =
          cyl->indices_of_assigned_points();
      Eigen::MatrixXd points(3, idx_assigned_points.size());
      for (unsigned i = 0; i < idx_assigned_points.size(); i++) {
        detected_points->indices.push_back(idx_assigned_points.at(i));
        Point_with_normal point_with_normal = outliers[idx_assigned_points[i]];
        Kernel::Point_3 p = point_with_normal.first;
        points.block<3, 1>(0, i) = Eigen::Vector3d(p.x(), p.y(), p.z());
      }
      Kernel::Vector_3 axis_temp = cyl->axis().to_vector();
      Eigen::Vector3d axis(axis_temp.x(), axis_temp.y(), axis_temp.z());
      axis.normalized();
      double radius = cyl->radius();
      points_shape_.push_back(points);
      ransac_normals_.push_back(Eigen::Vector3d::Zero());
      shape_id_.push_back(1);
      axis_.push_back(axis);
      radius_.push_back(radius);
    }
    it++;
  }

  extract.setInputCloud(meshing_points_);
  extract.setIndices(detected_points);
  extract.setNegative(true);
  extract.filter(*meshing_points_);
}

std::vector<Eigen::MatrixXd> *Model::getPointShapes() { return &points_shape_; }

std::vector<Eigen::Vector3d> *Model::getRansacNormals() {
  return &ransac_normals_;
}

std::vector<Eigen::Vector3d> *Model::getAxis() { return &axis_; }

std::vector<double> *Model::getRadius() { return &radius_; }

std::vector<int> *Model::getShapeIDs() { return &shape_id_; }

int Model::getOutlierCount() { return meshing_points_->size(); }

double Model::getMinDistance() { return std::sqrt(nn_dists_[0]); }

void Model::clearRansacShapes() {
  points_shape_.clear();
  ransac_normals_.clear();
  shape_id_.clear();
  axis_.clear();
  radius_.clear();
}

void Model::clearBuffer() { meshing_points_->clear(); }

}  // namespace cpt_reconstruction
}  // namespace cad_percept