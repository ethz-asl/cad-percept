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
#include <dynamic_reconfigure/server.h>
#include <cpt_ros/RMPConfigConfig.h>

class RMPTestNode {
 public:
  RMPTestNode(ros::NodeHandle nh) : nh_(nh) {
    ros::NodeHandle nh_private("~");
    pub_marker_ = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1, true);
    pub_mesh_3d_ = cad_percept::MeshModelPublisher(nh, "mesh_3d");
    pub_mesh_2d_ = cad_percept::MeshModelPublisher(nh, "mesh_2d");
    sub_spacenav_ = nh.subscribe("/spacenav/joy", 1, &RMPTestNode::callback, this);

    std::string path = nh_private.param<std::string>("off_path", "");
    double zero_angle = nh_private.param("zero_angle", 0.0);
    double zero_x = nh_private.param("zero_x", 0.0);
    double zero_y = nh_private.param("zero_y", 0.0);
    double zero_z = nh_private.param("zero_z", 0.0);

    "/home/mpantic/Desktop/rss/surfaces/crazy.off";
    bool success = cad_percept::cgal::MeshModel::create(path, &model_, true);

    Eigen::Vector3d zero;
    zero <<  zero_x, zero_y, zero_z;
    mapping_ = new cad_percept::planning::UVMapping(model_, zero, zero_angle);
    manifold_ = new cad_percept::planning::MeshManifoldInterface(model_, zero, zero_angle);
    pub_mesh_3d_.publish(model_);

    server_.setCallback(boost::bind(&RMPTestNode::config_callback, this, _1, _2));

    start_ << 0.0, 0.0, 0.0;
    target_ << 0.0, 0.0, 0.0;
  }

  void config_callback(cpt_ros::RMPConfigConfig &config, uint32_t level) {
    do_distance_ = config.do_distance;
    do_integrator_ = config.do_integrator;
    do_field_ = config.do_field;
    alpha = config.alpha;
    beta = config.beta;
    c = config.c;

    alpha_z = config.alpha_z;
    beta_z = config.beta_z;
    c_z = config.c_z;
  }

  void callback(const sensor_msgs::JoyConstPtr &joy) {

    using RMPG = rmp_core::PolicyContainer<3, 3, cad_percept::planning::MeshManifoldInterface>;
    RMPG policies(*manifold_);
    RMPG::Vector_x x_target3, x_target2, x_vec, x_dot;

    if (joy->buttons[0]) {
      Eigen::Vector3d start_tmp;
      start_tmp << joy->axes[0], joy->axes[1], fmax(0, joy->axes[2]);
      if (!mapping_->onManifold((Eigen::Vector2d) start_tmp.topRows<2>())) {
        return;
      }
      start_ = mapping_->pointUVHto3D(start_tmp);
    }
    if (joy->buttons[1]) {
      Eigen::Vector3d end_tmp;

      end_tmp << joy->axes[0], joy->axes[1], fmax(0, joy->axes[2]);
      if (!mapping_->onManifold((Eigen::Vector2d) end_tmp.topRows<2>())) {
        return;
      }
      target_ = end_tmp;
    }
    RMPG::Vector_q target_xyz = mapping_->pointUVHto3D(target_);

    rmp_core::Integrator<3, 3, cad_percept::planning::MeshManifoldInterface> integrator(policies);

    Eigen::Vector3d newpos = start_;
    integrator.resetTo(start_);

    Eigen::Matrix3d A{Eigen::Matrix3d::Identity()};
    Eigen::Matrix3d B{Eigen::Matrix3d::Identity()};
    A.diagonal() = Eigen::Vector3d({1.0, 1.0, 0.0});
    B.diagonal() = Eigen::Vector3d({0.0, 0.0, 1.0});
    rmp_core::SimpleTargetPolicy<3> pol2(target_, A, alpha, beta, c); // goes to manifold as quick as possible
    rmp_core::SimpleTargetPolicy<3> pol3({0.0, 0.0, 0.0}, B, alpha_z, beta_z, c_z); // stays along it
    policies.addPolicy(&pol3);
    policies.addPolicy(&pol2);

    visualization_msgs::Marker msg;
    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time::now();
    msg.type = visualization_msgs::Marker::LINE_LIST;
    msg.id = 0;
    msg.ns = "field";
    msg.action = visualization_msgs::Marker::ADD;
    msg.color.a = 1.0;
    msg.color.r = 1.0;
    msg.scale.x = 0.005;
    msg.pose.orientation.w = 1.0;

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

    double stride = 0.1;

    if(do_field_) {
      for (double x = -0.8; x < 0.8; x += stride) {
        for (double y = -0.8; y < 0.8; y += stride) {
          for (double z = 0.0; z < 0.2; z += stride) {

            x_dot << 0, 0, 0;
            x_vec << x, y, z;
            Eigen::Vector3d uv, xyz;
            uv << x, y, z;

            if (!mapping_->onManifold((Eigen::Vector2d) uv.topRows<2>())) {
              continue;
            }
            auto acc = policies.evaluate(x_vec, x_dot);
            double acc_norm = acc.norm();

            // get position in 3d!

            xyz = mapping_->pointUVHto3D(uv);

            acc *= 0.07;

            if (isnan(acc[0]) || isnan(acc[1]) || isnan(acc[2])) {
              // std::cout << "infinite value at" << uv << " " << xyz << std::endl;
              continue;
            }

            geometry_msgs::Point start;
            start.x = xyz.x();
            start.y = xyz.y();
            start.z = xyz.z();
            geometry_msgs::Point end;
            end.x = xyz.x() + acc.x();
            end.y = xyz.y() + acc.y();
            end.z = xyz.z() + acc.z();
            std_msgs::ColorRGBA color;
            color.a = 0.35;
            color.g = 1.0;
            color.b = 1.0;
            color.r = 1.0;

            msg.colors.push_back(color);
            msg.colors.push_back(color);

            msg.points.push_back(start);
            msg.points.push_back(end);

            msg_sphere.points.push_back(start);
            color.a = 0.75;
            msg_sphere.colors.push_back(color);
          }
        }
      }
    }

    visualization_msgs::Marker msg_path;
    msg_path.header.frame_id = "world";
    msg_path.header.stamp = ros::Time::now();
    msg_path.type = visualization_msgs::Marker::LINE_STRIP;
    msg_path.id = 2;
    msg_path.ns = "path";
    msg_path.action = visualization_msgs::Marker::ADD;
    msg_path.color.a = 0.95;
    msg_path.color.g = 1.0;
    msg_path.scale.x = 0.01;
    msg_path.scale.y = 0.01;
    msg_path.scale.z = 0.01;
    msg_path.pose.orientation.w = 1.0;

    double dt = 0.05;

    if(do_integrator_){
    for (double t = 0; t < 20.0; t += dt) {
      geometry_msgs::Point pos;
      pos.x = newpos.x();
      pos.y = newpos.y();
      pos.z = newpos.z();
      msg_path.points.push_back(pos);
      newpos = integrator.forwardIntegrate(dt);

      if (integrator.isDone()) {
        ROS_INFO_STREAM("Integrator finished after " << t << " s with a distance of " << integrator.totalDistance());

        // get geodesic distance
        if(do_distance_) {
        auto start_face = mapping_->nearestFace(start_);
        auto end_face = mapping_->nearestFace(target_xyz);


        cad_percept::cgal::Point start_xyz_pt(start_.x(),start_.y(),start_.z());
        cad_percept::cgal::Point end_xyz_pt(target_xyz.x(),target_xyz.y(),target_xyz.z());
        cad_percept::cgal::Point start_bary = (cad_percept::cgal::Point)start_face.second.toBarycentric(start_);
        cad_percept::cgal::Point end_bary = (cad_percept::cgal::Point)end_face.second.toBarycentric(target_xyz);



          double distance = model_->getGeodesicDistance(start_xyz_pt,
                                                        start_bary,
                                                        end_xyz_pt,
                                                        end_bary);
          ROS_INFO_STREAM("Geodesic distance = " << distance);
        }
        break;
      }
    }
    }

    visualization_msgs::MarkerArray msg_arr;
    msg_arr.markers.push_back(msg_path);
    msg_arr.markers.push_back(msg);
    msg_arr.markers.push_back(msg_sphere);
    pub_marker_.publish(msg_arr);

  }

 private:
  bool do_distance_  {false};
  bool do_field_  {false};
  bool do_integrator_  {false};
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
  ros::init(argc, argv, "rmp_test_node");
  ros::NodeHandle nh;

  RMPTestNode node(nh);

  ros::spin();
  return 0;
}
