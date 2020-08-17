#include <cgal_definitions/mesh_model.h>

#include <cpt_planning/coordinates/face_coords.h>
#include <cpt_planning/coordinates/uv_mapping.h>
#include <cpt_planning/interface/mesh_manifold_interface.h>
#include <cpt_ros/mesh_model_publisher.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <iostream>
#include <rmpflow/core/policy_container.h>
#include <rmpflow/geometry/linear_geometry.h>
#include <rmpflow/policies/simple_avoidance_policy.h>
#include <rmpflow/policies/simple_target_policy.h>
#include <rmpflow/eval/integrator.h>

#include <sensor_msgs/Joy.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include <cpt_ros/RMPConfigConfig.h>

class RMPCoordinateNode {
 public:
  RMPCoordinateNode(ros::NodeHandle nh) : nh_(nh) {
    ros::NodeHandle nh_private("~");
    pub_marker_ = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1, true);
    pub_mesh_3d_ = cad_percept::MeshModelPublisher(nh, "mesh_3d");
    pub_mesh_2d_ = cad_percept::MeshModelPublisher(nh, "mesh_2d");

    std::string path = nh_private.param<std::string>("off_path", "");
    double zero_angle = nh_private.param("zero_angle", 0.0);
    double zero_x = nh_private.param("zero_x", 0.0);
    double zero_y = nh_private.param("zero_y", 0.0);
    double zero_z = nh_private.param("zero_z", 0.0);

    bool success = cad_percept::cgal::MeshModel::create(path, &model_, true);

    Eigen::Vector3d zero;
    zero << zero_x, zero_y, zero_z;
    mapping_ = new cad_percept::planning::UVMapping(model_, zero, zero_angle);
    manifold_ = new cad_percept::planning::MeshManifoldInterface(model_, zero, zero_angle);
    pub_mesh_3d_.publish(model_);

    start_ << 0.0, 0.0, 0.0;
    target_ << 0.0, 0.0, 0.0;
  }

  void callback() {

    visualization_msgs::Marker msg_sphere;
    msg_sphere.header.frame_id = "world";
    msg_sphere.header.stamp = ros::Time::now();
    msg_sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    msg_sphere.id = 1;
    msg_sphere.ns = "field";
    msg_sphere.action = visualization_msgs::Marker::ADD;
    msg_sphere.color.a = 0.95;
    msg_sphere.color.r = 1.0;
    msg_sphere.scale.x = 0.01;
    msg_sphere.scale.y = 0.01;
    msg_sphere.scale.z = 0.01;
    msg_sphere.pose.orientation.w = 1.0;

    ROS_INFO_STREAM("COORDINATE START");
    for (int x = -10; x <= 10; x += 1) {
      for (int y = -1000; y <= 1000; y += 1) {
        Eigen::Vector3d uv, xyz;
        uv << x / 10.0, y / 1000.0, 0;

        if (!mapping_->onManifold((Eigen::Vector2d) uv.topRows<2>())) {
          continue;
        }

        xyz = mapping_->pointUVHto3D(uv);

        geometry_msgs::Point start;

        start.x = xyz.x();
        start.y = xyz.y();
        start.z = xyz.z();

        std_msgs::ColorRGBA color;
        if (x != 0) {
          color.r = 0.0;
          color.g = 0.0;
          color.b = 0.0;
          color.a = 1.0;
        } else {
          color.r = 1.0;
          color.g = 0.0;
          color.b = 0.0;
          color.a = 1.0;
        }
        std::cout <<
                  x << "\t" << y << "\t" << 0.0 << "\t" << xyz.x() << "\t" << xyz.y() << "\t" << xyz.z() << "\t" <<
                  color.r*255.0 << "\t" << color.g*255.0 << "\t" << color.b*255.0 << std::endl;
        msg_sphere.points.push_back(start);

        msg_sphere.colors.push_back(color);
      }
    }

    for (int x = -1000; x <= 1000; x += 1) {
      for (int y = -10; y <= 10; y += 1) {
        Eigen::Vector3d uv, xyz;
        uv << x / 1000.0, y / 10.0, 0;

        if (!mapping_->onManifold((Eigen::Vector2d) uv.topRows<2>())) {
          continue;
        }

        xyz = mapping_->pointUVHto3D(uv);

        geometry_msgs::Point start;

        start.x = xyz.x();
        start.y = xyz.y();
        start.z = xyz.z();

        std_msgs::ColorRGBA color;
        if (y != 0) {
          color.r = 0.0;
          color.g = 0.0;
          color.b = 0.0;
          color.a = 1.0;
        } else {
          color.r = 0.0;
          color.g = 1.0;
          color.b = 0.0;
          color.a = 1.0;
        }

        std::cout <<
                  x << "\t" << y << "\t" << 0.0 << "\t" << xyz.x() << "\t" << xyz.y() << "\t" << xyz.z() << "\t" <<
                  color.r*255.0<< "\t" << color.g*255.0 << "\t" << color.b*255.0<< std::endl;
        msg_sphere.points.push_back(start);

        msg_sphere.colors.push_back(color);
      }
    }

    ROS_INFO_STREAM("COORDINATE END");
    visualization_msgs::MarkerArray arr;
    arr.markers.push_back(msg_sphere);

    pub_marker_.publish(arr);

  }

 private:

  Eigen::Vector3d start_, target_;
  double alpha, beta, c;
  double alpha_z, beta_z, c_z;
  ros::NodeHandle nh_;
  ros::Publisher pub_marker_;
  ros::Subscriber sub_spacenav_;
  cad_percept::MeshModelPublisher pub_mesh_3d_;
  cad_percept::MeshModelPublisher pub_mesh_2d_;
  cad_percept::cgal::MeshModel::Ptr model_;
  cad_percept::planning::UVMapping *mapping_;
  cad_percept::planning::MeshManifoldInterface *manifold_;
  dynamic_reconfigure::Server<cpt_ros::RMPConfigConfig> server_;

};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "rmp_coordinate_node");
  ros::NodeHandle nh;

  RMPCoordinateNode node(nh);
  node.callback();

  ros::spin();
  return 0;
}
