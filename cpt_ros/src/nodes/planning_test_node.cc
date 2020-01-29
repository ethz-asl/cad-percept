#include <cgal_definitions/mesh_model.h>
#include <cpt_planning/coordinates/face_coords.h>
#include <cpt_planning/coordinates/uv_mapping.h>
#include <cpt_planning/interface/mesh_manifold_interface.h>
#include <cpt_ros/mesh_model_publisher.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <iostream>
class UVMappingTestNode {
 public:
  UVMappingTestNode(ros::NodeHandle nh) : nh_(nh) {
    pub_marker_ = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1, true);
    pub_mesh_3d_ = cad_percept::MeshModelPublisher(nh, "mesh_3d");
    pub_mesh_2d_ = cad_percept::MeshModelPublisher(nh, "mesh_2d");
    sub_spacenav_ = nh.subscribe("/spacenav/twist", 1, &UVMappingTestNode::callback, this);

    std::string path =
        "/home/mpantic/workspace/nccr_ws/src/cad-percept/cpt_utils/resources/simple_meshes/"
        "half_sphere.off";
    bool success = cad_percept::cgal::MeshModel::create(path, &model_, true);

    Eigen::Vector3d zero;
    zero << 0.0, 0.0, 0.707;
    mapping_ = new cad_percept::planning::UVMapping(model_, zero);
    manifold_ = new cad_percept::planning::MeshManifoldInterface(model_, zero);

    pub_mesh_3d_.publish(model_);
    pub_mesh_2d_.publish(mapping_->mesh_2d_);
  }

  void callback(const geometry_msgs::TwistConstPtr& twist) {
    Eigen::Vector3d point_uv, point_xyz, point_uv_reverse;
    point_uv << twist->linear.x, twist->linear.y, twist->linear.z;

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

  void publishMarkers(Eigen::Vector3d pt_uv,  Eigen::Vector3d pt_uv2, Eigen::Vector3d pt_xyz, Eigen::Vector3d vel_uv,
                      Eigen::Vector3d vel_xyz) {

    visualization_msgs::MarkerArray msg_array;

    visualization_msgs::Marker mrk_uv;
    mrk_uv.header.frame_id = "world";
    mrk_uv.header.stamp = ros::Time::now();
    mrk_uv.ns = "uv";
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
    point_uv2.x = pt_uv2.x();
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

    mrk_uv.points.push_back(point_uv);
    mrk_uv.points.push_back(point_uv2);
    mrk_uv.points.push_back(point_xyz);
    msg_array.markers.push_back(mrk_uv);

    visualization_msgs::Marker mrk_vel_uv;
    mrk_vel_uv.header.frame_id = "world";
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
    vel_uv += pt_uv;
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
    pub_marker_.publish(msg_array);
  }

 private:
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
