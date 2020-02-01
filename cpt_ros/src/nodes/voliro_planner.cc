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

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <dynamic_reconfigure/server.h>
#include <cpt_ros/RMPConfigConfig.h>
#include <mav_msgs/conversions.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
class VoliroPlanner {
 public:
  VoliroPlanner(ros::NodeHandle nh) : nh_(nh) {
    ros::NodeHandle nh_private("~");
    pub_marker_ = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1, true);
    pub_mesh_3d_ = cad_percept::MeshModelPublisher(nh, "mesh_3d");
    pub_mesh_2d_ = cad_percept::MeshModelPublisher(nh, "mesh_2d");
    sub_spacenav_ = nh.subscribe("/joy", 1, &VoliroPlanner::callback, this);
    pub_trajectory_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/ouzel/command/trajectory", 1);
    sub_odom_ = nh.subscribe("/ouzel/msf_core/odometry", 1, &VoliroPlanner::odom, this);
    std::string path = nh_private.param<std::string>("off_path", "");
    double zero_angle = nh_private.param("zero_angle", 0.0);
    double zero_x = nh_private.param("zero_x", 0.0);
    double zero_y = nh_private.param("zero_y", 0.0);
    double zero_z = nh_private.param("zero_z", 0.0);

    "/home/mpantic/Desktop/rss/surfaces/crazy.off";
    bool success = cad_percept::cgal::MeshModel::create(path, &model_, true);

    Eigen::Vector3d zero;
    zero << zero_x, zero_y, zero_z;
    mapping_ = new cad_percept::planning::UVMapping(model_, zero, zero_angle);
    manifold_ = new cad_percept::planning::MeshManifoldInterface(model_, zero, zero_angle);
    pub_mesh_3d_.publish(model_);

    server_.setCallback(boost::bind(&VoliroPlanner::config_callback, this, _1, _2));

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

  void odom(const nav_msgs::OdometryConstPtr &odom) {
    start_.x() = odom->pose.pose.position.x;
    start_.y() = odom->pose.pose.position.y;
    start_.z() = odom->pose.pose.position.z;

    start_uv_ = mapping_->point3DtoUVH(start_);
    ROS_WARN("GOT ODOM");
  }

  void callback(const sensor_msgs::JoyConstPtr &joy) {

    using RMPG = rmp_core::PolicyContainer<3, 3, cad_percept::planning::MeshManifoldInterface>;
    RMPG policies(*manifold_);
    RMPG::Vector_x x_target3, x_target2, x_vec, x_dot;

    /*
    if (joy->buttons[0]) {
      Eigen::Vector3d start_tmp;
      start_tmp << joy->axes[0], joy->axes[1], fmax(0, joy->axes[2]);
      if (!mapping_->onManifold((Eigen::Vector2d) start_tmp.topRows<2>())) {
        return;
      }
      start_ = mapping_->pointUVHto3D(start_tmp);
    }*/



    Eigen::Vector3d end_tmp;

    end_tmp << joy->axes[0]*0.5, joy->axes[1]*0.5, fmax(0, joy->axes[2]*0.5);
//    end_tmp += start_uv_;
    if (!mapping_->onManifold((Eigen::Vector2d) end_tmp.topRows<2>())) {
      return;
    }
    target_ = end_tmp;


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

    mav_msgs::EigenTrajectoryPoint::Vector trajectory;
    double dt = 0.01;
    for (double t = 0; t < 30.0; t += dt) {
      geometry_msgs::Point pos;
      pos.x = newpos.x();
      pos.y = newpos.y();
      pos.z = newpos.z();
      newpos = integrator.forwardIntegrate(dt);

      mav_msgs::EigenTrajectoryPoint pt;
      pt.time_from_start_ns = t * 1e9;
      integrator.getState(&pt.position_W, &pt.velocity_W, &pt.acceleration_W);

      //get jacobian of 3d space
      Eigen::Vector3d posuv = mapping_->point3DtoUVH(pt.position_W);
      Eigen::Matrix3d j = manifold_->J(posuv).inverse();
      j.col(0).normalize();
      j.col(1).normalize();
      j.col(2).normalize();

      Eigen::Matrix3d R;
      R.col(0) = -j.col(1);
      R.col(2) = j.col(2);
      R.col(1) = -R.col(0).cross(R.col(2));
      //Rotate around x
      Eigen::Affine3d rotx(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));
      //Eigen::Affine3d roty(Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitZ()));

      pt.orientation_W_B = Eigen::Quaterniond(R);
      trajectory.push_back(pt);

      if (integrator.isDone()) {
     //   ROS_INFO_STREAM("Integrator finished after " << t << " s with a distance of " << integrator.totalDistance());
        break;
      }
    }
    // Trajectory
    visualization_msgs::MarkerArray markers;
    double distance = 0.1; // Distance by which to seperate additional markers. Set 0.0 to disable.
    std::string frame_id = "world";
    mav_trajectory_generation::drawMavSampledTrajectory(trajectory, distance, frame_id, &markers);

    pub_marker_.publish(markers);
    if(joy->buttons[0]){  //Test
      trajectory_msgs::MultiDOFJointTrajectory msg;
      mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory, &msg);
      msg.header.frame_id = "world";
      msg.header.stamp = ros::Time::now();
      pub_trajectory_.publish(msg);
      published_ = true;
	ROS_WARN_STREAM("TRAJECTORY SENT");
    }

  }

 private:

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
  dynamic_reconfigure::Server<cpt_ros::RMPConfigConfig> server_;

};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "rmp_test_node");
  ros::NodeHandle nh;

  VoliroPlanner node(nh);

  ros::spin();
  return 0;
}
