#include <cpt_reconstruction/reconstruction_points_subscriber.h>
#include <cpt_reconstruction/reconstruction_preprocess_model.h>

#include <geometry_msgs/Vector3.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <cmath>
#include <sstream>
#include <string>
#include "cpt_reconstruction/coordinates.h"
#include "cpt_reconstruction/shape.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <fstream>

namespace cad_percept {
namespace cpt_reconstruction {
ReconstructionPointsSubscriber::ReconstructionPointsSubscriber(
    ros::NodeHandle nodeHandle1, ros::NodeHandle nodeHandle2,
    PreprocessModel* model)
    : nodeHandle1_(nodeHandle1),
      nodeHandle2_(nodeHandle2),
      model_(model),
      update_transformation_(true),
      counter_planes_(0),
      counter_cyl_(0),
      iteration_counter_(0) {
  subscriber1_ = nodeHandle1_.subscribe(
      "corrected_scan", 1000, &ReconstructionPointsSubscriber::messageCallback,
      this);
  publisher_ = nodeHandle2_.advertise<::cpt_reconstruction::shape>("ransac_shape", 1000);
  ros::spin();
}

void ReconstructionPointsSubscriber::messageCallback(
    const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {

  if (update_transformation_){
    tf::StampedTransform transform;
    tf_listener_.lookupTransform("/map", "/marker", ros::Time(0), transform);
    Eigen::Matrix3d rotation;
    tf::matrixTFToEigen(transform.getBasis(), rotation);
    Eigen::Vector3d translation;
    tf::vectorTFToEigen(transform.getOrigin(), translation);
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation.block(0, 0, 3, 3) = rotation;
    transformation.block(0, 3, 3, 1) = translation;
    transformation_inv_ = transformation.inverse();
    if ((transformation - transformation_).lpNorm<Eigen::Infinity>() < 0.0001) {
      update_transformation_ = false;
      ROS_INFO("Transformation changed!");
      transformation_ = transformation;
    }
  }

  // TODO: Transform model instead of points
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud_msg, *pcl_cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_transformed(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*pcl_cloud, *pcl_cloud_transformed,
                           transformation_inv_);

  ROS_INFO("[Subscriber] Received cloud with size: %d\n", pcl_cloud->size());

  std::ofstream file1;
  std::ofstream file2;
  file1.open("/home/philipp/Schreibtisch/outliers_ros.xyz", std::ofstream::app);
  file2.open("/home/philipp/Schreibtisch/outliers_ros_full.xyz",
             std::ofstream::app);
  for (int i = 0; i < pcl_cloud_transformed->size(); i++) {
    double x = (*pcl_cloud_transformed)[i].x;
    double y = (*pcl_cloud_transformed)[i].y;
    double z = (*pcl_cloud_transformed)[i].z;
    pcl::PointXYZ pcl_p(x, y, z);
    model_->queryTree(pcl_p);
    if (model_->getMinDistance() >= 0.05) {
      model_->addOutlier(pcl_p);
      file1 << x << " " << y << " " << z << "\n";
    }
    file2 << x << " " << y << " " << z << "\n";
  }
  file1.close();
  file2.close();


  ROS_INFO("[Subscriber] Outlier count: %d\n", model_->getOutlierCount());
  if (model_->getOutlierCount() > 30000) {
    model_->clearRansacShapes();
    //model_->applyFilter();
    model_->efficientRANSAC();
    //model_->SACSegmentation();

    std::vector<Eigen::MatrixXd>* points_shape = model_->getPointShapes();
    std::vector<Eigen::MatrixXd>* normals_shape = model_->getNormalShapes();
    std::vector<Eigen::Vector3d>* ransac_normal = model_->getRansacNormals();
    std::vector<int>* shapes_ids = model_->getShapeIDs();

    // Publish mesh to mesh_gereration
    for (int i = 0; i < shapes_ids->size(); i++) {
      std::vector<geometry_msgs::Vector3> pub_points;
      std::vector<geometry_msgs::Vector3> pub_normals;
      for (int j = 0; j < points_shape->at(i).cols(); j++) {
        geometry_msgs::Vector3 p;
        geometry_msgs::Vector3 n;
        p.x = (*points_shape)[i](0, j);
        p.y = (*points_shape)[i](1, j);
        p.z = (*points_shape)[i](2, j);
        n.x = (*normals_shape)[i](0, j);
        n.y = (*normals_shape)[i](1, j);
        n.z = (*normals_shape)[i](2, j);
        pub_points.push_back(p);
        pub_normals.push_back(n);
      }
      Eigen::Vector3d ransac_n_temp = (*ransac_normal).at(i);
      geometry_msgs::Vector3 ransac_n;
      ransac_n.x = ransac_n_temp.x();
      ransac_n.y = ransac_n_temp.y();
      ransac_n.z = ransac_n_temp.z();

      ::cpt_reconstruction::shape shape_msg;
      shape_msg.points_msg = pub_points;
      shape_msg.normals_msg = pub_normals;
      shape_msg.ransac_normal = ransac_n;
      shape_msg.id = shapes_ids->at(i);
      publisher_.publish(shape_msg);
    }

    // Save points to file
    // TODO: Remove
    for (int i = 0; i < shapes_ids->size(); i++) {
      std::ofstream file_shape;
      if (shapes_ids->at(i) == 0) {
        std::string result = "/home/philipp/Schreibtisch/Shapes/shape_" +
                             std::to_string(counter_planes_) + "_plane.xyz";
        counter_planes_++;
        file_shape.open(result);
      } else {
        std::string result = "/home/philipp/Schreibtisch/Shapes/shape_" +
                             std::to_string(counter_cyl_) + "_cyl.xyz";
        counter_cyl_++;
        file_shape.open(result);
      }
      for (int j = 0; j < points_shape->at(i).cols(); j++) {
        Eigen::Vector3d vec = (*points_shape)[i].col(j);
        file_shape << vec.x() << " " << vec.y() << " " << vec.z() << " " << (*ransac_normal).at(i).x() << " " << (*ransac_normal).at(i).y() << " " << (*ransac_normal).at(i).z() << "\n";
      }
    }

    if ((iteration_counter_ >= 5) && (iteration_counter_ % 5 == 0)) {
      model_->clearBuffer();
    }

    iteration_counter_++;
  }
}
}  // namespace cpt_reconstruction
}  // namespace cad_percept