#include <cgal_definitions/mesh_model.h>
#include <cpt_planning/coordinates/face_coords.h>
#include <cpt_planning/coordinates/uv_mapping.h>
#include <cpt_planning/interface/mesh_manifold_interface.h>
#include <cpt_ros/mesh_model_publisher.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/Joy.h>
#include <iostream>
#include <cgal_definitions/mesh_model.h>

class UVMappingTestNode {
 public:
  UVMappingTestNode(ros::NodeHandle nh) : nh_(nh) {
    pub_marker_ = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1, true);
    pub_mesh_3d_ = cad_percept::MeshModelPublisher(nh, "mesh_3d");
    pub_mesh_2d_ = cad_percept::MeshModelPublisher(nh, "mesh_2d");
    sub_spacenav_ = nh.subscribe("/spacenav/joy", 1, &UVMappingTestNode::callback, this);
    ros::NodeHandle nh_private("~");

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
    pub_mesh_2d_.publish(mapping_->mesh_2d_, "uv");

    callback();
  }

  void callback(const sensor_msgs::JoyConstPtr &joy) {
    Eigen::Vector3d point_uv, point_xyz, point_uv_reverse;

    point_uv << joy->axes[0]*0.5, joy->axes[1]*0.5, fmax(0, joy->axes[2]*0.5);
    point_xyz = mapping_->pointUVHto3D(point_uv);
    point_uv_reverse = mapping_->point3DtoUVH(point_xyz);

    std::cout << "uv1:\t" << point_uv.x() << "\t" << point_uv.y() << "\t" << point_uv.z() << std::endl;
    std::cout << "xy2:\t" << point_xyz.x() << "\t" << point_xyz.y() << "\t" << point_xyz.z() << std::endl;
    std::cout << "uv2:\t" << point_uv_reverse.x() << "\t" << point_uv_reverse.y() << "\t" << point_uv_reverse.z() << std::endl;
    std::cout << "dif:\t" <<  (point_uv - point_uv_reverse).norm() << std::endl;
    std::cout << std::endl;
    // get velocity in uv f.e.xample
    Eigen::Vector3d vel_uv;
    vel_uv << 0.0, 0.0, 1.0;

    Eigen::Vector3d vel_xyz = manifold_->J(point_uv).inverse() * vel_uv;
    publishMarkers(point_uv, point_uv_reverse, point_xyz, vel_uv, vel_xyz);

  }

  void callback() {


    msg_coordinates.header.frame_id = "world";
    msg_coordinates.header.stamp = ros::Time::now();
    msg_coordinates.type = visualization_msgs::Marker::SPHERE_LIST;
    msg_coordinates.id = 1;
    msg_coordinates.ns = "coordinates";
    msg_coordinates.action = visualization_msgs::Marker::ADD;
    msg_coordinates.color.a = 0.95;
    msg_coordinates.color.r = 1.0;
    msg_coordinates.scale.x = 0.01;
    msg_coordinates.scale.y = 0.01;
    msg_coordinates.scale.z = 0.01;
    msg_coordinates.pose.orientation.w = 1.0;

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
        msg_coordinates.points.push_back(start);

        msg_coordinates.colors.push_back(color);
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
        msg_coordinates.points.push_back(start);

        msg_coordinates.colors.push_back(color);
      }
    }



  }
  void publishMarkers(Eigen::Vector3d pt_uv,  Eigen::Vector3d pt_uv2, Eigen::Vector3d pt_xyz, Eigen::Vector3d vel_uv,
                      Eigen::Vector3d vel_xyz) {

    visualization_msgs::MarkerArray msg_array;

    visualization_msgs::Marker mrk_uv;
    mrk_uv.header.frame_id = "world";
    mrk_uv.header.stamp = ros::Time::now();
    mrk_uv.ns = "world";
    mrk_uv.id = 0;
    mrk_uv.type = visualization_msgs::Marker::SPHERE_LIST;
    mrk_uv.action = visualization_msgs::Marker::ADD;
    mrk_uv.scale.x = 0.1;
    mrk_uv.scale.y = 0.1;
    mrk_uv.scale.z = 0.1;
    mrk_uv.color.a = 1.0;
    mrk_uv.color.r = 1.0;

    geometry_msgs::Point point_uv;
    point_uv.x = pt_uv.x();
    point_uv.y = pt_uv.y();
    point_uv.z = pt_uv.z();
    geometry_msgs::Point point_uv2;
    point_uv2.x = pt_uv2.x()+2.0;
    point_uv2.y = pt_uv2.y();
    point_uv2.z = pt_uv2.z();

    std_msgs::ColorRGBA red, green;
    red.a = 1.0;
    red.r = 1.0;
    green.a= 1.0;
    green.g = 1.0;

    geometry_msgs::Point point_xyz;
    point_xyz.x = pt_xyz.x();
    point_xyz.y = pt_xyz.y();
    point_xyz.z = pt_xyz.z();

    mrk_uv.colors.push_back(red);
    mrk_uv.colors.push_back(green);
    mrk_uv.colors.push_back(red);

    //mrk_uv.points.push_back(point_uv);
    mrk_uv.points.push_back(point_uv2);
    mrk_uv.points.push_back(point_xyz);
    msg_array.markers.push_back(mrk_uv);

    visualization_msgs::Marker mrk_vel_uv;
    mrk_vel_uv.header.frame_id = "uv";
    mrk_vel_uv.header.stamp = ros::Time::now();
    mrk_vel_uv.ns = "uv";
    mrk_vel_uv.id = 1;
    mrk_vel_uv.type = visualization_msgs::Marker::ARROW;
    mrk_vel_uv.action = visualization_msgs::Marker::ADD;
    mrk_vel_uv.scale.x = 0.05;
    mrk_vel_uv.scale.y = 0.07;

    mrk_vel_uv.color.a = 1.0;
    mrk_vel_uv.color.r = 0.0;
    mrk_vel_uv.color.r = 1.0;

    geometry_msgs::Point point_vel_uv;
    vel_uv += pt_uv2;
    point_vel_uv.x = vel_uv.x();
    point_vel_uv.y = vel_uv.y();
    point_vel_uv.z = vel_uv.z();
    mrk_vel_uv.points.push_back(point_uv);
    mrk_vel_uv.points.push_back(point_vel_uv);
    msg_array.markers.push_back(mrk_vel_uv);

    visualization_msgs::Marker mrk_vel_xyz;
    mrk_vel_xyz.header.frame_id = "world";
    mrk_vel_xyz.header.stamp = ros::Time::now();
    mrk_vel_xyz.ns = "uv";
    mrk_vel_xyz.id = 2;
    mrk_vel_xyz.type = visualization_msgs::Marker::ARROW;
    mrk_vel_xyz.action = visualization_msgs::Marker::ADD;
    mrk_vel_xyz.scale.x = 0.05;
    mrk_vel_xyz.scale.y = 0.07;

    mrk_vel_xyz.color.a = 1.0;
    mrk_vel_xyz.color.r = 0.0;
    mrk_vel_xyz.color.r = 1.0;

    geometry_msgs::Point point_vel_xyz;
    vel_xyz += pt_xyz;
    point_vel_xyz.x = vel_xyz.x();
    point_vel_xyz.y = vel_xyz.y();
    point_vel_xyz.z = vel_xyz.z();

    mrk_vel_xyz.points.push_back(point_xyz);
    mrk_vel_xyz.points.push_back(point_vel_xyz);

    msg_array.markers.push_back(mrk_vel_xyz);
    msg_array.markers.push_back(msg_coordinates);

    pub_marker_.publish(msg_array);
  }

 private:
  visualization_msgs::Marker msg_coordinates;
  ros::NodeHandle nh_;
  ros::Publisher pub_marker_;
  ros::Subscriber sub_spacenav_;
  cad_percept::MeshModelPublisher pub_mesh_3d_;
  cad_percept::MeshModelPublisher pub_mesh_2d_;
  cad_percept::cgal::MeshModel::Ptr model_;
  cad_percept::planning::UVMapping* mapping_;
  cad_percept::planning::MeshManifoldInterface* manifold_;
};

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "collision_manifold_test_node");
  ros::NodeHandle nh;

  UVMappingTestNode node(nh);

  ros::spin();
  return 0;
}
