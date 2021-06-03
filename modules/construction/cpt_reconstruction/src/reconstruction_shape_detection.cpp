#include <cpt_reconstruction/reconstruction_model.h>
#include <cpt_reconstruction/reconstruction_shape_detection.h>

namespace cad_percept {
namespace cpt_reconstruction {
ShapeDetection::ShapeDetection(ros::NodeHandle nodeHandle1,
                               ros::NodeHandle nodeHandle2, Model *model)
    : nodeHandle1_(nodeHandle1),
      nodeHandle2_(nodeHandle2),
      model_(model),
      update_transformation_(true),
      counter_planes_(0),
      counter_cyl_(0),
      iteration_counter_(0) {
  nodeHandle1.getParam("SensorType", SENSOR_TYPE_);
  nodeHandle1.getParam("TransformationMatrix", TRANSFORMATION_VEC_);
  nodeHandle1.getParam("ModelTolerance", MODEL_TOLERANCE_);
  nodeHandle1.getParam("OutlierCount", OUTLIER_COUNT_);
  nodeHandle1.getParam("UseBuffer", USE_BUFFER_);
  nodeHandle1.getParam("ClearBufferAfterIteration",
                       CLEAR_BUFFER_AFTER_ITERATION_);
  nodeHandle1.getParam("OutputAllPointsFile", ALL_POINTS_PATH_);
  nodeHandle1.getParam("OutputOutlierPointsFile", OUTLIER_POINTS_PATH_);

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      TRANSFORMATION_(i, j) = TRANSFORMATION_VEC_.at(i * 4 + j);
    }
  }

  subscriber1_ = nodeHandle1_.subscribe("corrected_scan", 1000,
                                        &ShapeDetection::messageCallback, this);
  publisher_ =
      nodeHandle2_.advertise<::cpt_reconstruction::shape>("ransac_shape", 1000);
  ros::spin();
}

void ShapeDetection::messageCallback(
    const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
  if (update_transformation_) {
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

  tf::StampedTransform robot_pos_stamp;
  tf_listener_.lookupTransform("/map", "/lidar", ros::Time(0), robot_pos_stamp);
  Eigen::Vector3d robot_pos;
  tf::vectorTFToEigen(robot_pos_stamp.getOrigin(), robot_pos);
  Eigen::Vector4d translation_h(robot_pos.x(), robot_pos.y(), robot_pos.z(),
                                1.0);
  robot_pos = (transformation_inv_ * translation_h).block<3, 1>(0, 0);

  // ROS_INFO("Robot x: %f\n", robot_pos.x());
  // ROS_INFO("Robot y: %f\n", robot_pos.y());
  // ROS_INFO("Robot z: %f\n", robot_pos.z());

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
  file1.open(ALL_POINTS_PATH_, std::ofstream::app);
  file2.open(OUTLIER_POINTS_PATH_, std::ofstream::app);
  for (int i = 0; i < pcl_cloud_transformed->size(); i++) {
    double x = (*pcl_cloud_transformed)[i].x;
    double y = (*pcl_cloud_transformed)[i].y;
    double z = (*pcl_cloud_transformed)[i].z;
    pcl::PointXYZ pcl_p(x, y, z);
    model_->queryTree(pcl_p);
    if (model_->getMinDistance() >= MODEL_TOLERANCE_) {
      model_->addOutlier(pcl_p);
      file1 << x << " " << y << " " << z << "\n";
    }
    file2 << x << " " << y << " " << z << "\n";
  }
  file1.close();
  file2.close();

  ROS_INFO("[Subscriber] Outlier count: %d\n", model_->getOutlierCount());
  if (model_->getOutlierCount() > OUTLIER_COUNT_) {
    model_->clearRansacShapes();
    model_->applyFilter();
    model_->efficientRANSAC();
    // model_->SACSegmentation();

    std::vector<Eigen::MatrixXd> *points_shape = model_->getPointShapes();
    std::vector<Eigen::Vector3d> *ransac_normal = model_->getRansacNormals();
    std::vector<Eigen::Vector3d> *axis = model_->getAxis();
    std::vector<double> *radius = model_->getRadius();
    std::vector<int> *shapes_ids = model_->getShapeIDs();

    // Publish mesh to mesh_gereration
    for (int i = 0; i < shapes_ids->size(); i++) {
      std::vector<geometry_msgs::Vector3> pub_points;
      for (int j = 0; j < points_shape->at(i).cols(); j++) {
        geometry_msgs::Vector3 p;
        p.x = (*points_shape)[i](0, j);
        p.y = (*points_shape)[i](1, j);
        p.z = (*points_shape)[i](2, j);
        pub_points.push_back(p);
      }
      Eigen::Vector3d ransac_n_temp = (*ransac_normal).at(i);
      geometry_msgs::Vector3 ransac_n;
      ransac_n.x = ransac_n_temp.x();
      ransac_n.y = ransac_n_temp.y();
      ransac_n.z = ransac_n_temp.z();

      geometry_msgs::Vector3 pub_robot_position;
      pub_robot_position.x = robot_pos.x();
      pub_robot_position.y = robot_pos.y();
      pub_robot_position.z = robot_pos.z();

      geometry_msgs::Vector3 pub_axis;
      Eigen::Vector3d axis_temp = (*axis).at(i);
      pub_axis.x = axis_temp.x();
      pub_axis.y = axis_temp.y();
      pub_axis.z = axis_temp.z();

      ::cpt_reconstruction::shape shape_msg;
      shape_msg.points_msg = pub_points;
      shape_msg.ransac_normal = ransac_n;
      shape_msg.robot_position = pub_robot_position;
      shape_msg.axis = pub_axis;
      shape_msg.radius = radius->at(i);
      shape_msg.id = shapes_ids->at(i);
      publisher_.publish(shape_msg);
    }

    if ((!USE_BUFFER_) || (CLEAR_BUFFER_AFTER_ITERATION_ >= 2 &&
                           CLEAR_BUFFER_AFTER_ITERATION_ % 2 == 0)) {
      model_->clearBuffer();
    }

    iteration_counter_++;
  }
}
}  // namespace cpt_reconstruction
}  // namespace cad_percept