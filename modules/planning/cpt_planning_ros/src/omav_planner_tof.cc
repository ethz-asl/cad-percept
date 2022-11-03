#include <cpt_planning_ros/omav_planner_tof.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

OmavPlanner::OmavPlanner(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : nh_(nh), nh_private_(nh) {
  pol_orientation_ = std::make_shared<SO3TargetPolicy>();
  Eigen::Vector4d target_quat(0.258, 0.426, 0.234, 0.835);
  pol_orientation_->setTarget(target_quat);
  pol_orientation_->setA(Eigen::Matrix4d::Identity());
  pol_orientation_->setTuning(1, 16, 0.05);

  sub_curr_ref_ =
      nh.subscribe("/stork/command/current_reference", 1, &OmavPlanner::setPointCallback, this);
  debug_trigger_ = nh.advertiseService("run", &OmavPlanner::debugCallback, this);

  pub_setpoint_ =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/stork/command/trajectory", 1);
}

bool OmavPlanner::debugCallback(std_srvs::Empty::Request &request,
                                std_srvs::Empty::Response &response) {
  mav_msgs::EigenTrajectoryPointVector newtraject;
  newtraject.resize(100);

  runOrientationPlanner(&newtraject);
  for (auto &pt : newtraject) {
    std::cout << pt.orientation_W_B.x() << " " << pt.orientation_W_B.y() << " "
              << pt.orientation_W_B.z() << " " << pt.orientation_W_B.w() << " " << std::endl;

    pt.position_W = T_odom_body_.translation();
  }

  trajectory_msgs::MultiDOFJointTrajectory  msg;
  mav_msgs::msgMultiDofJointTrajectoryFromEigen(newtraject, &msg);
  pub_setpoint_.publish(msg);
  return true;
}
void OmavPlanner::runOrientationPlanner(mav_msgs::EigenTrajectoryPointVector *traject) {
  // get current orientation state
  Eigen::Quaterniond q(T_odom_body_.rotation());
  Eigen::Vector4d w_vec = {w_.x(), w_.y(), w_.z(), 0};
  Eigen::Vector4d wdot_vec = {wdot_.x(), wdot_.y(), wdot_.z(), 0};

  So3Integrator so3Integrator;
  so3Integrator.resetTo(q, w_vec, wdot_vec);
  SO3GeometryPtr so3Geometry = std::make_shared<SO3Geometry>();

  for (int i = 0; i < traject->size(); i++) {
    so3Integrator.integrateStep({pol_orientation_}, so3Geometry, dt_ * 10);

    Eigen::Vector4d w_out, wdot_out;
    so3Integrator.getState(&traject->operator[](i).orientation_W_B, &w_out, &wdot_out);

    traject->operator[](i).angular_velocity_W = T_odom_body_.rotation().inverse() * w_out.topRows<3>(0);
    traject->operator[](i).angular_acceleration_W = T_odom_body_.rotation().inverse() * wdot_out.topRows<3>(0);
  }
}

void OmavPlanner::runPositionPlanner(mav_msgs::EigenTrajectoryPointVector* traject) {
  pol_body_->setTarget({1.0, 0.0, 0.0});


  Eigen::Vector3d pos_body, v_body, a_body;
  pos_body = Eigen::Vector3d::Zero(); // always at zero in body!
  v_body = T_odom_body_.rotation().inverse() * v_;
  a_body = T_odom_body_.rotation().inverse() * vdot_;
  R3Integrator r3Integrator;
  R3GeometryPtr r3Geometry = std::make_shared<R3Geometry>();




}

void OmavPlanner::setWeights(double pos_global, double pos_z, double pos_body) {
  pol_global_->setA(pos_global * Eigen::Matrix3d::Identity());
  pol_cam_z_->setA(pos_z * Eigen::Vector3d(0, 0, 1).asDiagonal());
  pol_body_->setA(pos_body * Eigen::Vector3d(1, 1, 0).asDiagonal());
}

void OmavPlanner::normalCallback(const geometry_msgs::PoseStampedPtr &vctr) {
  // calculate desired quaternion
  Eigen::Vector4d quat;
  pol_orientation_->setTarget(quat);

  // calculate ??
  double distance;
  // torolololo do pol_z_.setTarget()
}

void OmavPlanner::presenterCallback(const geometry_msgs::Vector3StampedPtr &vctr) {
  // set x/y targets in bodyframe
  double x = std::clamp(vctr->vector.x / 100.0, -0.1, 0.1);
  double y = std::clamp(vctr->vector.y / 100.0, -0.1, 0.1);
  pol_body_->setTarget({x, y, 0.0});
}

void OmavPlanner::setPointCallback(const trajectory_msgs::MultiDOFJointTrajectoryPtr &traj) {
  auto latest = traj.get()->points.at(0);
  mav_msgs::EigenTrajectoryPoint latest_eigen;
  mav_msgs::eigenTrajectoryPointFromMsg(latest, &latest_eigen);

  // todo: test if that all is halfway correct...
  T_odom_body_.translation() = latest_eigen.position_W;
  T_odom_body_.linear() = latest_eigen.orientation_W_B.toRotationMatrix();
  v_ = latest_eigen.velocity_W;
  w_ = latest_eigen.angular_velocity_W;
  vdot_ = latest_eigen.acceleration_W;
  wdot_ = latest_eigen.angular_acceleration_W;

  state_valid_ = true;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "omav_planner_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  OmavPlanner node(nh, nh_private);

  ros::spin();

  return 0;
}
