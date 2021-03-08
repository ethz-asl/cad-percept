#include <cgal_definitions/mesh_model.h>
#include <cpt_planning/coordinates/face_coords.h>
#include <cpt_planning/coordinates/uv_mapping.h>
#include <cpt_planning/interface/mesh_manifold_interface.h>
#include <cpt_planning_ros/RMPConfigConfig.h>
#include <cpt_ros/mesh_model_publisher.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <mav_msgs/conversions.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mavros_msgs/PositionTarget.h>
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
class CSVFieldNode {
 public:
  CSVFieldNode(ros::NodeHandle nh) : nh_(nh) {
    ros::NodeHandle nh_private("~");
    sub_pose_array_ = nh.subscribe("fieldconfig", 1, &CSVFieldNode::posearray, this);
    sub_odom_ = nh.subscribe("odometry", 1000, &CSVFieldNode::odom, this);
    sub_setpoint_ = nh.subscribe("/xoliro/mavros/setpoint_raw/target_local", 1000,
                                 &CSVFieldNode::setpoint, this);

    std::string path = nh_private.param<std::string>("off_path", "");

    cad_percept::cgal::MeshModel::create(path, &model_, true);

    double zero_angle = nh_private.param("zero_angle", 0.0);
    double zero_x = nh_private.param("zero_x", 0.0);
    double zero_y = nh_private.param("zero_y", 0.0);
    double zero_z = nh_private.param("zero_z", 0.0);

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

    createSampling();
  }

  void createSampling() {
    double duv = 0.05;
    for (double u = -0.95; u <= 0.95; u += duv) {
      for (double v = -0.95; v <= 0.95; v += duv) {
        Eigen::Vector2d pos(u, v);
        if (mapping_->onManifold(pos)) {
          sampling_positions_uv_.push_back(pos);
        }
      }
    }
  }

  Eigen::Vector3d toEigen(geometry_msgs::Point pt) { return {pt.x, pt.y, pt.z}; }

  void posearray(const geometry_msgs::PoseArrayConstPtr &poses) {
    last_target_uv_ = toEigen(poses->poses[0].position);
    last_target_xyz_ = toEigen(poses->poses[1].position);
    param_pol_a_ = toEigen(poses->poses[4].position);
    param_pol_b_ = toEigen(poses->poses[5].position);
    active = true;
    std::cout << "Received policy" << std::endl;
  }
  void setpoint(const mavros_msgs::PositionTargetConstPtr &target) {
    last_ctrl_target_.x() = target->position.x;
    last_ctrl_target_.y() = target->position.y;
    last_ctrl_target_.z() = target->position.z;
    // odom->header.stamp - last_output_ >= ros::Duration(0.2)
    // output 5 per second
    if (active) {
      std::cout << "outputing field " << fieldnum << std::endl;
      outputField();
      // last_output_ = odom->header.stamp;
    }
  }
  void odom(const nav_msgs::OdometryConstPtr &odom) {
    last_odom_.x() = odom->pose.pose.position.x;
    last_odom_.y() = odom->pose.pose.position.y;
    last_odom_.z() = odom->pose.pose.position.z;

    last_vel_.x() = odom->twist.twist.linear.x;
    last_vel_.y() = odom->twist.twist.linear.y;
    last_vel_.z() = odom->twist.twist.linear.z;

    last_odom_orient_.x() = odom->pose.pose.orientation.x;
    last_odom_orient_.y() = odom->pose.pose.orientation.y;
    last_odom_orient_.z() = odom->pose.pose.orientation.z;
    last_odom_orient_.w() = odom->pose.pose.orientation.w;
  }

  Eigen::Quaterniond getOrientation(Eigen::Vector3d pos_w) {
    Eigen::Vector3d posuv = mapping_->point3DtoUVH(pos_w);
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

    Eigen::Quaterniond q(R);
    return q;
  }

  void outputField() {
    using RMPG = cad_percept::planning::MeshManifoldInterface;
    using LinSpace = rmpcpp::Space<3>;
    using TargetPolicy = rmpcpp::SimpleTargetPolicy<LinSpace>;
    using Integrator = rmpcpp::TrapezoidalIntegrator<TargetPolicy, RMPG>;

    // Set up policies
    Eigen::Matrix3d A{Eigen::Matrix3d::Identity()};
    Eigen::Matrix3d B{Eigen::Matrix3d::Identity()};
    A.diagonal() = Eigen::Vector3d({1.0, 1.0, 0.0});
    B.diagonal() = Eigen::Vector3d({0.0, 0.0, 1.0});
    TargetPolicy pol2(last_target_uv_, A, param_pol_a_.x(), param_pol_a_.y(),
                      param_pol_a_.z());  // goes to manifold as quick as possible
    TargetPolicy pol3(last_target_uv_, B, param_pol_b_.x(), param_pol_b_.y(),
                      param_pol_b_.z());  // stays along it
    std::vector<TargetPolicy *> policies;
    policies.push_back(&pol2);
    policies.push_back(&pol3);

    // get current position in UV
    Eigen::Vector3d last_odom_uvh = mapping_->point3DtoUVH(last_odom_);
    Eigen::Vector3d last_ctrl_trgt_uvh = mapping_->point3DtoUVH(last_ctrl_target_);
    std::ofstream logfile;
    logfile.open("/tmp/field/field_" + std::to_string(fieldnum) + ".log");
    Eigen::Quaterniond last_target_orient_ = getOrientation(last_target_xyz_);
    logfile << last_target_xyz_.x() << "\t" << last_target_xyz_.y() << "\t" << last_target_xyz_.z()
            << "\t" << last_target_orient_.w() << "\t" << last_target_orient_.x() << "\t"
            << last_target_orient_.y() << "\t" << last_target_orient_.z() << std::endl;


    logfile << last_odom_.x() << "\t" << last_odom_.y() << "\t" << last_odom_.z() << "\t"
            << last_odom_orient_.w() << "\t" << last_odom_orient_.x() << "\t"
            << last_odom_orient_.y() << "\t" << last_odom_orient_.z() << std::endl;

    Eigen::Quaterniond last_ctrl_orient_ = getOrientation(last_ctrl_target_);

    logfile << last_ctrl_target_.x() << "\t" << last_ctrl_target_.y() << "\t"
            << last_ctrl_target_.z() << "\t" << last_ctrl_orient_.w() << "\t"
            << last_ctrl_orient_.x() << "\t" << last_ctrl_orient_.y() << "\t"
            << last_ctrl_orient_.z() << std::endl;

    std::cout << last_target_uv_.z() - last_odom_uvh.z() << std::endl;
    for (auto pos2d : sampling_positions_uv_) {
      Eigen::Vector3d pos_uvh(pos2d.x(), pos2d.y(), last_ctrl_trgt_uvh.z());

      Eigen::Vector3d pos_xyz = mapping_->pointUVHto3D(pos_uvh);

      RMPG::StateQ current_stateQ{pos_xyz, last_vel_};
      RMPG::StateX current_stateX = manifold_->convertToX(current_stateQ);

      // evaluate all policy and get new accelerations
      std::vector<TargetPolicy::PValue> evaluated_policies;
      for (TargetPolicy *policy : policies) {
        evaluated_policies.push_back(manifold_->at(current_stateX).pull(*policy));
      }
      Eigen::Vector3d acc_b = TargetPolicy::PValue::sum(evaluated_policies).f_;
      logfile << pos_xyz.x() << "\t" << pos_xyz.y() << "\t" << pos_xyz.z() << "\t" << acc_b.x()
              << "\t" << acc_b.y() << "\t" << acc_b.z() << "\t" << 0.0 << std::endl;
    }

    logfile.close();
    fieldnum++;
  }

 private:
  bool active = false;
  int fieldnum = 0;
  Eigen::Vector3d last_odom_, last_vel_, last_target_uv_, last_target_xyz_, param_pol_a_,
      param_pol_b_, last_ctrl_target_;
  Eigen::Quaterniond last_odom_orient_;
  double alpha, beta, c;
  double alpha_z, beta_z, c_z;
  ros::NodeHandle nh_;
  ros::Subscriber sub_pose_array_, sub_odom_, sub_setpoint_;
  tf::TransformListener listener_;
  ros::Time last_output_;
  cad_percept::cgal::MeshModel::Ptr model_;
  cad_percept::planning::UVMapping *mapping_;
  cad_percept::planning::MeshManifoldInterface *manifold_;
  std::vector<Eigen::Vector2d> sampling_positions_uv_;
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "csvfield_node");
  ros::NodeHandle nh;

  CSVFieldNode node(nh);

  ros::spin();

  return 0;
}
