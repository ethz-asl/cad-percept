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
class RMPTestNode {
 public:
  RMPTestNode(ros::NodeHandle nh) : nh_(nh) {
    pub_marker_ = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1, true);
    pub_mesh_3d_ = cad_percept::MeshModelPublisher(nh, "mesh_3d");
    pub_mesh_2d_ = cad_percept::MeshModelPublisher(nh, "mesh_2d");

    std::string path =
        "/home/mpantic/workspace/nccr_ws/src/cad-percept/cpt_utils/resources/simple_meshes/"
        "half_sphere.off";
    bool success = cad_percept::cgal::MeshModel::create(path, &model_, true);

    Eigen::Vector3d zero;
    zero << 0.0, 0.0, 0.707;
    mapping_ = new cad_percept::planning::UVMapping(model_, zero);
    manifold_ = new cad_percept::planning::MeshManifoldInterface(model_, zero);
  }

  void callback() {
    pub_mesh_3d_.publish(model_);
    using RMPG = rmp_core::PolicyContainer<3, 3, cad_percept::planning::MeshManifoldInterface>;
    RMPG policies(*manifold_);
    RMPG::Vector_x x_target3,x_target2, x_vec, x_dot;
    x_target3 << 0, -0.25, 0;
    x_target2 << 0.3, 0.3, 0;
    rmp_core::SimpleTargetPolicy<3> pol3(x_target3);
    rmp_core::SimpleTargetPolicy<3> pol2(x_target2);
    policies.addPolicy(&pol2);
    policies.addPolicy(&pol3);

    visualization_msgs::Marker msg;
    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time::now();
    msg.type = visualization_msgs::Marker::LINE_LIST;
    msg.id = 0;
    msg.ns = "field";
    msg.action = visualization_msgs::Marker::ADD;
    msg.color.a = 1.0;
    msg.color.r = 1.0;
    msg.scale.x = 0.001;
    msg.pose.orientation.w = 1.0;

    visualization_msgs::Marker msg_sphere;
    msg_sphere.header.frame_id = "world";
    msg_sphere.header.stamp = ros::Time::now();
    msg_sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    msg_sphere.id = 1;
    msg_sphere.ns = "field";
    msg_sphere.action = visualization_msgs::Marker::ADD;
    msg_sphere.color.a = 1.0;
    msg_sphere.color.r = 1.0;
    msg_sphere.scale.x = 0.005;
    msg_sphere.scale.y = 0.005;
    msg_sphere.scale.z = 0.005;
    msg_sphere.pose.orientation.w = 1.0;

    double stride = 0.01;

    for (double x = -0.5; x < 0.5; x += stride) {
      for (double y = -0.5; y < 0.5; y += stride) {

        x_dot << 0, 0, 0;
        x_vec << x, y, 0;

        auto acc = policies.evaluate(x_vec, x_dot);
        double acc_norm = acc.norm();

        // get position in 3d!
        Eigen::Vector2d uv;
        uv << x, y;
        auto faces = mapping_->nearestFace(uv);
        Eigen::Vector3d xyz = faces.first.translateTo(faces.second, uv);

        acc *= 0.3;

        if (isnan(acc[0]) || isnan(acc[1]) || isnan(acc[2])) {
          std::cout << "infinite value at" << uv << " " << xyz << std::endl;
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
        color.a = 0.75;
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
    visualization_msgs::MarkerArray msg_arr;
    msg_arr.markers.push_back(msg);
    msg_arr.markers.push_back(msg_sphere);
    pub_marker_.publish(msg_arr);

  }

 private:
  ros::NodeHandle nh_;
  ros::Publisher pub_marker_;
  ros::Subscriber sub_spacenav_;
  cad_percept::MeshModelPublisher pub_mesh_3d_;
  cad_percept::MeshModelPublisher pub_mesh_2d_;
  cad_percept::cgal::MeshModel::Ptr model_;
  cad_percept::planning::UVMapping *mapping_;
  cad_percept::planning::MeshManifoldInterface *manifold_;
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "rmp_test_node");
  ros::NodeHandle nh;

  RMPTestNode node(nh);
  node.callback();

  ros::spin();
  return 0;
}
