#include <cpt_reconstruction/reconstruction_points_subscriber.h>
#include <cpt_reconstruction/reconstruction_preprocess_model.h>

#include <geometry_msgs/Vector3.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <cmath>
#include <sstream>
#include <string>
#include "cpt_reconstruction/coordinates.h"
#include "cpt_reconstruction/shape.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <fstream>

namespace cad_percept {
namespace cpt_reconstruction {
ReconstructionPointsSubscriber::ReconstructionPointsSubscriber(
    ros::NodeHandle nodeHandle1, ros::NodeHandle nodeHandle2,
    PreprocessModel* model)
    : nodeHandle1_(nodeHandle1), nodeHandle2_(nodeHandle2), model_(model), update_transformation_(true) {
  subscriber1_ = nodeHandle1_.subscribe("corrected_scan", 10, &ReconstructionPointsSubscriber::messageCallback, this);
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

    if ( (transformation - transformation_).norm() < 0.005){
      update_transformation_ = false;
      ROS_INFO("Transformation fixed");
    }
    transformation_ = transformation;
    transformation_inv_ = transformation.inverse();
  }

  //TODO: Transform model instead of points
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud_msg, *pcl_cloud);
  pcl::transformPointCloud(*pcl_cloud, *pcl_cloud_transformed, transformation_inv_);

  ROS_INFO("[Subscriber] Received cloud with size: %d\n", pcl_cloud->size());

  std::ofstream file1;
  std::ofstream file2;
  file1.open("/home/philipp/Schreibtisch/outliers_ros.xyz", std::ofstream::app);
  file2.open("/home/philipp/Schreibtisch/outliers_ros_full.xyz", std::ofstream::app);
  for(int i = 0; i < pcl_cloud_transformed->size(); i++){
    double x = (*pcl_cloud_transformed)[i].x;
    double y = (*pcl_cloud_transformed)[i].y;
    double z = (*pcl_cloud_transformed)[i].z;
    pcl::PointXYZ pcl_p(x, y, z);
    model_->queryTree(pcl_p);
    model_->getMinDistance();
    if (model_->getMinDistance() >= 0.015) {
      file1 << x << " " << y << " " << z << "\n";
    }
    file2 << x << " " << y << " " << z << "\n";
  }
  file1.close();
  file2.close();

  /*
  pcl::PointXYZ p(msg.x, msg.y, msg.z);
  model_->queryTree(p);
  float min_dist = model_->getMinDistance();
  if (std::abs(min_dist) >= 0.015) {
    model_->addOutlier(msg.idx, p);
    ROS_INFO("[Subscriber] Received %f %f %f with min_dist %f and nr: %d\n",
             msg.x, msg.y, msg.z, min_dist, model_->getOutlierCount());
    if (model_->getOutlierCount() > 500) {
      model_->clearRansacShapes();
      model_->efficientRANSAC();

      std::vector<Eigen::MatrixXd>* points_shape = model_->getPointShapes();
      std::vector<int>* shapes_ids = model_->getShapeIDs();

      for (int i = 0; i < shapes_ids->size(); i++) {
        std::vector<geometry_msgs::Vector3> pub_vectors;
        for (int j = 0; j < points_shape->at(i).cols(); j++) {
          geometry_msgs::Vector3 vec;
          vec.x = (*points_shape)[i](0, j);
          vec.y = (*points_shape)[i](1, j);
          vec.z = (*points_shape)[i](2, j);
          pub_vectors.push_back(vec);
        }
        ::cpt_reconstruction::shape msg;
        msg.vectors = pub_vectors;
        msg.id = shapes_ids->at(i);
        publisher_.publish(msg);
      }
    }
  }*/
}
}  // namespace cpt_reconstruction
}  // namespace cad_percept