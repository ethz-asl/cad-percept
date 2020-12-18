#include <cgal_definitions/mesh_model.h>
#include <cpt_planning/coordinates/face_coords.h>
#include <cpt_planning/coordinates/uv_mapping.h>
#include <cpt_planning/interface/mesh_manifold_interface.h>
#include <cpt_planning_ros/RMPConfigConfig.h>
#include <cpt_ros/mesh_model_publisher.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Twist.h>
#include <mav_msgs/conversions.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <nav_msgs/Odometry.h>
#include <rmpcpp/core/space.h>
#include <rmpcpp/core/state.h>
#include <rmpcpp/eval/trapezoidal_integrator.h>
#include <rmpcpp/policies/simple_target_policy.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/MarkerArray.h>

#include <iostream>
class VoliroPlanner {
 public:
  VoliroPlanner(ros::NodeHandle nh) : nh_(nh) {
    ros::NodeHandle nh_private("~");
    pub_marker_ = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1, true);
    pub_mesh_3d_ = cad_percept::MeshModelPublisher(nh, "mesh_3d");
    pub_mesh_2d_ = cad_percept::MeshModelPublisher(nh, "mesh_2d");
    sub_spacenav_ = nh.subscribe("/joy", 1, &VoliroPlanner::callback, this);
    pub_trajectory_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("cmd_trajectory", 1);
    sub_odom_ = nh.subscribe("odometry", 1, &VoliroPlanner::odom, this);
    std::string path = nh_private.param<std::string>("off_path", "");
    double zero_angle = nh_private.param("zero_angle", 0.0);
    double zero_x = nh_private.param("zero_x", 0.0);
    double zero_y = nh_private.param("zero_y", 0.0);
    double zero_z = nh_private.param("zero_z", 0.0);

    cad_percept::cgal::MeshModel::create(path, &model_, true);

    // transform model
    Eigen::Affine3d tf_eigen;
    tf::StampedTransform transform;

    listener_.waitForTransform("mesh", "world", ros::Time(0), ros::Duration(3.0));

    listener_.lookupTransform("mesh", "world", ros::Time(0), transform);
    tf::transformTFToEigen(transform, tf_eigen);
    model_->transform(
        cad_percept::cgal::eigenTransformationToCgalTransformation(tf_eigen.matrix()));

    Eigen::Vector3d zero;
    zero << zero_x, zero_y, zero_z;
    mapping_ = new cad_percept::planning::UVMapping(model_, zero, zero_angle);
    manifold_ = new cad_percept::planning::MeshManifoldInterface(model_, zero, zero_angle);
    pub_mesh_3d_.publish(model_);
    pub_mesh_2d_.publish(mapping_->mesh_2d_, "mesh2d");

    server_.setCallback(boost::bind(&VoliroPlanner::config_callback, this, _1, _2));

    start_ << 0.0, 0.0, 0.0;
    target_ << 0.0, 0.0, 0.0;
  }

  void config_callback(cpt_planning_ros::RMPConfigConfig &config, uint32_t level) {
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

  void odom(const nav_msgs::OdometryConstPtr &odom) {
    start_.x() = odom->pose.pose.position.x;
    start_.y() = odom->pose.pose.position.y;
    start_.z() = odom->pose.pose.position.z;

    start_uv_ = mapping_->point3DtoUVH(start_);
  }

  void publishMarker(Eigen::Vector3d pos_mesh, Eigen::Vector2d pos_mesh2d) {
    visualization_msgs::Marker marker_mesh;
    marker_mesh.header.frame_id = "world";
    marker_mesh.header.stamp = ros::Time();
    marker_mesh.ns = "meshmarker";
    marker_mesh.id = 0;
    marker_mesh.type = visualization_msgs::Marker::SPHERE_LIST;
    marker_mesh.action = visualization_msgs::Marker::ADD;
    marker_mesh.pose.position.x = 0.0;
    marker_mesh.pose.position.y = 0.0;
    marker_mesh.pose.position.z = 0.0;
    marker_mesh.pose.orientation.x = 0.0;
    marker_mesh.pose.orientation.y = 0.0;
    marker_mesh.pose.orientation.z = 0.0;
    marker_mesh.pose.orientation.w = 1.0;
    marker_mesh.scale.x = 0.25;
    marker_mesh.scale.y = 0.25;
    marker_mesh.scale.z = 0.25;
    marker_mesh.color.a = 1.0;  // Don't forget to set the alpha!
    marker_mesh.color.r = 0.0;
    marker_mesh.color.g = 1.0;
    marker_mesh.color.b = 0.0;

    geometry_msgs::Point pt3d;
    pt3d.x = pos_mesh.x();
    pt3d.y = pos_mesh.y();
    pt3d.z = pos_mesh.z();
    marker_mesh.points.push_back(pt3d);

    visualization_msgs::Marker marker_mesh2d;
    marker_mesh2d.header.frame_id = "mesh2d";
    marker_mesh2d.header.stamp = ros::Time();
    marker_mesh2d.ns = "meshmarker2d";
    marker_mesh2d.id = 0;
    marker_mesh2d.type = visualization_msgs::Marker::SPHERE_LIST;
    marker_mesh2d.action = visualization_msgs::Marker::ADD;
    marker_mesh2d.pose.position.x = 0.0;
    marker_mesh2d.pose.position.y = 0.0;
    marker_mesh2d.pose.position.z = 0.0;
    marker_mesh2d.pose.orientation.x = 0.0;
    marker_mesh2d.pose.orientation.y = 0.0;
    marker_mesh2d.pose.orientation.z = 0.0;
    marker_mesh2d.pose.orientation.w = 1.0;
    marker_mesh2d.scale.x = 0.25;
    marker_mesh2d.scale.y = 0.25;
    marker_mesh2d.scale.z = 0.25;
    marker_mesh2d.color.a = 1.0;  // Don't forget to set the alpha!
    marker_mesh2d.color.r = 0.0;
    marker_mesh2d.color.g = 1.0;
    marker_mesh2d.color.b = 0.0;

    geometry_msgs::Point pt2d;
    pt2d.x = pos_mesh2d.x();
    pt2d.y = pos_mesh2d.y();
    marker_mesh.points.push_back(pt2d);

    visualization_msgs::MarkerArray msg;
    msg.markers.push_back(marker_mesh);
    msg.markers.push_back(marker_mesh2d);
    pub_marker_.publish(msg);
  }

  void callback(const sensor_msgs::JoyConstPtr &joy) {
    using RMPG = cad_percept::planning::MeshManifoldInterface;
    using LinSpace = rmpcpp::Space<3>;
    using TargetPolicy = rmpcpp::SimpleTargetPolicy<LinSpace>;
    using Integrator = rmpcpp::TrapezoidalIntegrator<TargetPolicy, RMPG>;
    RMPG::VectorX x_target3, x_target2, x_vec, x_dot;
    Eigen::Vector3d end_tmp;

    end_tmp << joy->axes[0] * 0.5, -joy->axes[1] * 0.5, 0.0;
    end_tmp.x() += start_uv_.x();
    end_tmp.y() += start_uv_.y();
    end_tmp.z() = start_uv_.z();

    // get z offset
    if (joy->axes[5] < 0.0) {
      end_tmp.z() -= joy->axes[5] * 0.2;  // negative values -> subtract (want to go further away
    } else if (joy->axes[2] < 0.0) {
      end_tmp.z() = std::max(0.0, end_tmp.z() + joy->axes[2] * 0.2);
    }

    end_tmp.topRows<2>() =
        (Eigen::Vector2d)mapping_->clipToManifold((Eigen::Vector2d)end_tmp.topRows<2>());

    target_ = end_tmp;

    RMPG::VectorQ target_xyz = mapping_->pointUVHto3D(target_);

    publishMarker(target_xyz, end_tmp.topRows<2>());

    Integrator integrator;

    Eigen::Vector3d newpos = start_;

    Eigen::Matrix3d A{Eigen::Matrix3d::Identity()};
    Eigen::Matrix3d B{Eigen::Matrix3d::Identity()};
    A.diagonal() = Eigen::Vector3d({1.0, 1.0, 0.0});
    B.diagonal() = Eigen::Vector3d({0.0, 0.0, 1.0});
    TargetPolicy pol2(target_, A, alpha, beta, c);        // goes to manifold as quick as possible
    TargetPolicy pol3(target_, B, alpha_z, beta_z, c_z);  // stays along it
    std::vector<TargetPolicy *> policies;
    policies.push_back(&pol2);
    policies.push_back(&pol3);

    mav_msgs::EigenTrajectoryPoint::Vector trajectory;
    integrator.resetTo(newpos);
    double dt = 0.01;
    for (double t = 0; t < 15.0; t += dt) {
      geometry_msgs::Point pos;
      pos.x = newpos.x();
      pos.y = newpos.y();
      pos.z = newpos.z();
      newpos = integrator.forwardIntegrate(policies, manifold_, dt);

      mav_msgs::EigenTrajectoryPoint pt;
      pt.time_from_start_ns = t * 1e9;
      integrator.getState(&pt.position_W, &pt.velocity_W, &pt.acceleration_W);

      // get jacobian of 3d space
      Eigen::Vector3d posuv = mapping_->point3DtoUVH(pt.position_W);
      Eigen::Matrix3d j = manifold_->J(posuv).inverse();
      j.col(0).normalize();
      j.col(1).normalize();
      j.col(2).normalize();

      Eigen::Matrix3d R;

      // Important: only use normal + one other axis
      // otherwise transformation could be a non-rectangular frame and an invalid
      // transform.
      R.col(0) = -j.col(2);                  // negative normal becomes x
      R.col(2) = -j.col(1);                  // one axis becomes z
      R.col(1) = -R.col(0).cross(R.col(2));  // do the rest

      // add relative angle
      if (joy->buttons[10]) {
        relative_alpha_ = 0.0;
        relative_beta_ = 0.0;
      } else {
        relative_alpha_ =
            std::clamp(relative_alpha_ + joy->axes[4] * 0.05, -M_PI / 2.0, M_PI / 2.0);
        relative_beta_ = std::clamp(relative_beta_ + joy->axes[3] * 0.05, -M_PI / 2.0, M_PI / 2.0);
      }

      Eigen::AngleAxisd alpha_rot(relative_alpha_, Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd beta_rot(relative_beta_, Eigen::Vector3d::UnitZ());
      std::cout << relative_alpha_ << std::endl;
      std::cout << relative_beta_ << std::endl;
      pt.orientation_W_B = Eigen::Quaterniond(beta_rot * alpha_rot * R);

      trajectory.push_back(pt);

      if (integrator.isDone()) {
        ROS_INFO_STREAM("Integrator finished after " << t << " s with a distance of "
                                                     << integrator.totalDistance());
        break;
      }
    }
    std::cout << std::endl << std::endl;
    // Trajectory
    visualization_msgs::MarkerArray markers;
    double distance = 0.1;  // Distance by which to seperate additional markers. Set 0.0 to disable.
    std::string frame_id = "world";

    mav_trajectory_generation::drawMavSampledTrajectory(trajectory, distance, frame_id, &markers);
    // pub_marker_.publish(markers);
    if (joy->buttons[5] && zeroed_) {  // Test
      trajectory_msgs::MultiDOFJointTrajectory msg;
      mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory, &msg);
      msg.header.frame_id = "world";
      msg.header.stamp = ros::Time::now();
      pub_trajectory_.publish(msg);
      published_ = true;
      ROS_WARN_STREAM("TRAJECTORY SENT");
      zeroed_ = false;
    }

    if (!joy->buttons[5]) {
      zeroed_ = true;
    }
  }

 private:
  double relative_alpha_ = 0.0;
  double relative_beta_ = 0.0;

  bool zeroed_ = false;
  tf::TransformListener listener_;
  bool published_{false};
  bool do_distance_{false};
  bool do_field_{false};
  bool do_integrator_{false};
  Eigen::Vector3d start_, start_uv_, target_;
  double alpha, beta, c;
  double alpha_z, beta_z, c_z;
  ros::NodeHandle nh_;
  ros::Publisher pub_marker_;
  ros::Publisher pub_trajectory_;
  ros::Subscriber sub_spacenav_, sub_odom_;
  cad_percept::MeshModelPublisher pub_mesh_3d_;
  cad_percept::MeshModelPublisher pub_mesh_2d_;
  cad_percept::cgal::MeshModel::Ptr model_;
  cad_percept::planning::UVMapping *mapping_;
  cad_percept::planning::MeshManifoldInterface *manifold_;
  dynamic_reconfigure::Server<cpt_planning_ros::RMPConfigConfig> server_;
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "rmp_test_node");
  ros::NodeHandle nh;

  VoliroPlanner node(nh);

  ros::spin();

  return 0;
}
