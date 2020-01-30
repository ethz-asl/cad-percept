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

class RMPZDistanceNode {
 public:
  RMPZDistanceNode(ros::NodeHandle nh) : nh_(nh) {
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
    server_.setCallback(boost::bind(&RMPZDistanceNode::config_callback, this, _1, _2));

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

    ROS_INFO_STREAM("START");

    for (int x = -25; x <= 25; x += 1) {
      /* find lowest and largest y that's still on manifold */

      int ymin = 5000;
      int ymax = -5000;

      for (int y = -50; y <= 50; y += 1) {
        Eigen::Vector3d end_uv, end_xyz;
        end_uv << x / 25.0, y / 50.0, 0;
        if (mapping_->onManifold((Eigen::Vector2d) end_uv.topRows<2>())) {
          if (y < ymin) {
            ymin = y;
          }
          if (y > ymax) {
            ymax = y;
          }
        }
      }
      if (ymin == 5000 || ymax == -5000) {
        continue;
      }

      std::cerr << x << std::endl;

      Eigen::Vector3d end_uv, end_xyz, start_uv, start_xyz;
      start_uv << x / 25.0, ymin / 50.0, 0;
      end_uv << x / 25.0, ymax / 50.0, 0;

      if (!mapping_->onManifold((Eigen::Vector2d) start_uv.topRows<2>())) {
        continue;
      }
      if (!mapping_->onManifold((Eigen::Vector2d) end_uv.topRows<2>())) {
        continue;
      }

      start_xyz = mapping_->pointUVHto3D(start_uv);
      end_xyz = mapping_->pointUVHto3D(end_uv);
      using RMPG = rmp_core::PolicyContainer<3, 3, cad_percept::planning::MeshManifoldInterface>;
      RMPG policies(*manifold_);


      rmp_core::Integrator<3, 3, cad_percept::planning::MeshManifoldInterface> integrator(policies);

      Eigen::Matrix3d A{Eigen::Matrix3d::Identity()};
      Eigen::Matrix3d B{Eigen::Matrix3d::Identity()};
      A.diagonal() = Eigen::Vector3d({1.0, 1.0, 0.0});
      B.diagonal() = Eigen::Vector3d({0.0, 0.0, 1.0});
      rmp_core::SimpleTargetPolicy<3> pol2(end_uv, A, alpha, beta, c); // goes to manifold as quick as possible
      rmp_core::SimpleTargetPolicy<3> pol3({0.0, 0.0, 0.0}, B, alpha_z, beta_z, c_z); // stays along it
      policies.addPolicy(&pol3);
      policies.addPolicy(&pol2);

      integrator.resetTo(start_xyz);

      Eigen::Vector3d current_pos = start_xyz;
      double dt = 0.01;
      double distance = -1.0;
      for (double t = 0; t < 20.0; t += dt) {

        Eigen::Vector3d current_pos_uv= mapping_->point3DtoUVH(current_pos);
        std::cout <<
                  current_pos.x() << "\t" << current_pos.y() << "\t" << current_pos.z()
                  << "\t" << current_pos_uv.z()<< "\t" << x <<std::endl;

        current_pos = integrator.forwardIntegrate(dt);

        if (integrator.isDone()) {
          distance = integrator.totalDistance();
          break;
        }
      }





    }

    ROS_INFO_STREAM("END");

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

  RMPZDistanceNode node(nh);
  for(int i=0; i< 10; i++){
  ros::spinOnce();
  std::cin.get();
  }
  ros::spinOnce();
  std::cerr << "start" << std::endl;
  node.callback();
  std::cerr << "done" << std::endl;

  return 0;
}
