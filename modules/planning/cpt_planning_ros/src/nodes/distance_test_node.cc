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

class RMPDistanceNode {
 public:
  RMPDistanceNode(ros::NodeHandle nh) : nh_(nh) {
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
    server_.setCallback(boost::bind(&RMPDistanceNode::config_callback, this, _1, _2));

    start_ << 0.0, 0.0, 0.0;
    target_ << 0.0, 0.0, 0.0;
  }

  void config_callback(cpt_ros::RMPConfigConfig &config, uint32_t level) {
    ROS_WARN_STREAM("SET CONFIG");
    alpha = config.alpha;
    beta = config.beta;
    c = config.c;

    alpha_z = config.alpha_z;
    beta_z = config.beta_z;
    c_z = config.c_z;
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
    using RMPG = rmp_core::PolicyContainer<3, 3, cad_percept::planning::MeshManifoldInterface>;
    RMPG policies(*manifold_);
    Eigen::Vector3d target_;
    target_ << 0.0, 0.0, 0.0;

    RMPG::Vector_q target_xyz = mapping_->pointUVHto3D(target_);

    rmp_core::Integrator<3, 3, cad_percept::planning::MeshManifoldInterface> integrator(policies);

    Eigen::Matrix3d A{Eigen::Matrix3d::Identity()};
    Eigen::Matrix3d B{Eigen::Matrix3d::Identity()};
    A.diagonal() = Eigen::Vector3d({1.0, 1.0, 0.0});
    B.diagonal() = Eigen::Vector3d({0.0, 0.0, 1.0});
    rmp_core::SimpleTargetPolicy<3> pol2(target_, A, alpha, beta, c); // goes to manifold as quick as possible
    rmp_core::SimpleTargetPolicy<3> pol3({0.0, 0.0, 0.0}, B, alpha_z, beta_z, c_z); // stays along it
    policies.addPolicy(&pol3);
    policies.addPolicy(&pol2);

    ROS_INFO_STREAM("START");
    for (int x = -50; x <= 50; x += 1) {
      for (int y = -50; y <= 50; y += 1) {

        std::cerr << x << "\ " << y << std::endl;
        if (x == 0 && y == 0) {
          continue;
        }
        Eigen::Vector3d end_uv, end_xyz;
        end_uv << x / 50.0, y / 50.0, 0;

        if (!mapping_->onManifold((Eigen::Vector2d) end_uv.topRows<2>())) {
          continue;
        }

        end_xyz = mapping_->pointUVHto3D(end_uv);

        integrator.resetTo(end_xyz);

        Eigen::Vector3d current_pos;
        double dt = 0.01;
        double distance = -1.0;
        for (double t = 0; t < 30.0; t += dt) {

          current_pos = integrator.forwardIntegrate(dt);

          if (integrator.isDone()) {
            distance = integrator.totalDistance();
            break;
          }
        }

        cad_percept::cgal::Point start_xyz_pt(target_xyz.x(), target_xyz.y(), target_xyz.z());
        cad_percept::cgal::Point end_xyz_pt(end_xyz.x(), end_xyz.y(), end_xyz.z());

        double distance_geo = model_->getGeodesicDistance(start_xyz_pt,
                                                          start_xyz_pt,
                                                          end_xyz_pt,
                                                          end_xyz_pt);

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
                  x << "\t" << y << "\t" << 0.0 << "\t" << end_xyz.x() << "\t" << end_xyz.y() << "\t" << end_xyz.z()
                  << "\t" << distance << "\t" << distance_geo << "\t" << distance / distance_geo << std::endl;

        geometry_msgs::Point pos;
        pos.x = end_xyz.x();
        pos.y = end_xyz.y();
        pos.z = end_xyz.z();
        msg_sphere.points.push_back(pos);
        msg_sphere.colors.push_back(color);
      }
    }

    ROS_INFO_STREAM("END");
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

  RMPDistanceNode node(nh);
  ros::spinOnce();
  std::cin.get();
  ros::spinOnce();
  std::cerr << "start"<< std::endl;
  node.callback();
  std::cerr << "done"<< std::endl;


  return 0;
}
