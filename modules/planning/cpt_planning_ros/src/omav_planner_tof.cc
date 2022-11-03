#include <cpt_planning_ros/omav_planner_tof.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_srvs/Empty.h>

#include <cmath>

OmavPlanner::OmavPlanner(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : nh_(nh), nh_private_(nh) {
  pol_orientation_ = std::make_shared<SO3TargetPolicy>();

  pol_orientation_->setA(Eigen::Matrix4d::Identity());
  pol_orientation_->setTuning(1, 16, 0.05);

  pol_body_ = std::make_shared<R3TargetPolicy>();
  pol_body_->setA(Eigen::Vector3d(1, 1, 1).asDiagonal());
  pol_body_->setTuning(1, 2, 0.05);


  pol_body_ptr_ = std::make_shared<R3TargetPolicy>();
  pol_body_ptr_->setA(Eigen::Vector3d(0.1, 0.1, 0).asDiagonal());
  pol_body_ptr_->setTuning(1, 2, 0.05);

  sub_curr_ref_ =
      nh.subscribe("/stork/command/current_reference", 1, &OmavPlanner::setPointCallback, this);
  debug_trigger_ = nh.advertiseService("run", &OmavPlanner::debugCallback, this);

  pub_setpoint_ =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/stork/command/trajectory", 1);

  sub_joystick_ = nh.subscribe("/joy", 1, &OmavPlanner::joystickCallback, this);

  sub_pose_= nh.subscribe("/pose", 1, &OmavPlanner::normalCallback, this);

  sub_presenter_ = nh.subscribe("/presenter",1, &OmavPlanner::presenterCallback, this);


  Eigen::Affine3d T_cam_imu;
  T_cam_imu.matrix().row(0) << -0.01505397360614591, 0.9990103805857247, -0.04185256695391397, -0.10923818913817124;
  T_cam_imu.matrix().row(1) << -0.9995505126949042, -0.01612103339047266, -0.02527617165985309, 0.018915155905933494;
  T_cam_imu.matrix().row(2) << -0.025925864499000972, 0.04145324793535066, 0.998804023713156, -0.07059681620459023;
  T_cam_imu.matrix().row(3) << 0.0, 0.0, 0.0, 1.0;
  T_body_cam_ = T_cam_imu.inverse();
}

void OmavPlanner::run() {
  mav_msgs::EigenTrajectoryPointVector newtraject;
  newtraject.resize(2);
  runPositionPlanner(&newtraject);
  runOrientationPlanner(&newtraject);
  trajectory_msgs::MultiDOFJointTrajectory msg;

  mav_msgs::msgMultiDofJointTrajectoryFromEigen(newtraject, &msg);

  if (state_valid_) {
    pub_setpoint_.publish(msg);
  }
}

bool OmavPlanner::debugCallback(std_srvs::Empty::Request &request,
                                std_srvs::Empty::Response &response) {
  mav_msgs::EigenTrajectoryPointVector newtraject;
  newtraject.resize(2);

  for (auto &pt : newtraject) {
    std::cout << pt.orientation_W_B.x() << " " << pt.orientation_W_B.y() << " "
              << pt.orientation_W_B.z() << " " << pt.orientation_W_B.w() << " " << std::endl;

    pt.position_W = T_odom_body_.translation();
  }
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

    traject->operator[](i).angular_velocity_W =
        T_odom_body_.rotation().inverse() * w_out.topRows<3>(0);
    traject->operator[](i).angular_acceleration_W =
        T_odom_body_.rotation().inverse() * wdot_out.topRows<3>(0);
  }
}

void OmavPlanner::runPositionPlanner(mav_msgs::EigenTrajectoryPointVector *traject) {
  Eigen::Vector3d pos_body, v_body, a_body;
  pos_body = Eigen::Vector3d::Zero();  // always at zero in body!
  v_body = T_odom_body_.rotation().inverse() * v_;
  R3Integrator r3Integrator;
  R3RotatedGeometryPtr r3Geometry = std::make_shared<R3RotatedGeometry>();
  R3GeometryPtr r3LinearGeometry = std::make_shared<R3Geometry>();
  r3Geometry->setRotation(T_odom_body_.rotation().inverse());

  r3Integrator.resetTo(T_odom_body_.translation(), v_);

  for (int i = 0; i < traject->size(); i++) {
    // pull body policy

    auto policy_w = r3Geometry->at({pos_body, v_body}).pull(*pol_body_);
    auto policy_ptr_w = r3Geometry->at({pos_body, v_body}).pull(*pol_body_ptr_);

    r3Integrator.integrateStep({}, r3LinearGeometry, dt_, {policy_w,policy_ptr_w});

    r3Integrator.getState(&traject->operator[](i).position_W, &traject->operator[](i).velocity_W,
                          &traject->operator[](i).acceleration_W);
  }

}

void OmavPlanner::setWeights(double pos_global, double pos_z, double pos_body) {
  pol_global_->setA(pos_global * Eigen::Matrix3d::Identity());
  pol_cam_z_->setA(pos_z * Eigen::Vector3d(0, 0, 1).asDiagonal());
  pol_body_->setA(pos_body * Eigen::Vector3d(1, 1, 0).asDiagonal());
}

void OmavPlanner::joystickCallback(const sensor_msgs::JoyConstPtr &joy) {
  std::cout << "joy " << std::endl;
  double deg2rad = M_PI / 180.0;
  double pitch = joy.get()->axes[1] * 10 * deg2rad;
  double roll = -joy.get()->axes[0] * 10 * deg2rad;

  Eigen::Quaterniond q(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));

  q = T_odom_body_.rotation() * q;

  if (joy.get()->buttons[0]) {
    pol_orientation_->setTarget({q.x(), q.y(), q.z(), q.w()});
  }else if (joy.get()->buttons[1]) {
    q = desired_normal_orientation_;
    pol_orientation_->setTarget({q.x(), q.y(), q.z(), q.w()});
  } else {
    double z = joy.get()->axes[2];
    if (abs(z) < 0.25) {
      z = 0.0;
    } else {
      z *= 0.01;
    }


    pol_body_->setTarget({joy.get()->axes[1] * 0.1, joy.get()->axes[0] * 0.1, z});
  }

  if(joy.get()->buttons[8]){
    pol_body_ptr_->setA(Eigen::Vector3d(1.0,1.0,0.0).asDiagonal());
  }else{
    pol_body_ptr_->setA(Eigen::Vector3d(0.0,0.0,0.0).asDiagonal());
    ptr_target_ = Eigen::Vector3d::Zero();
    pol_body_ptr_->setTarget(ptr_target_);

  }
}

void OmavPlanner::normalCallback(const geometry_msgs::PoseStampedPtr &vctr) {
  // calculate desired quaternion
  Eigen::Quaterniond normal_cam(vctr.get()->pose.orientation.w, vctr.get()->pose.orientation.x,
                                vctr.get()->pose.orientation.y, vctr.get()->pose.orientation.z);

  Eigen::Vector3d dist_cam(vctr.get()->pose.position.x, vctr.get()->pose.position.y,
                           vctr.get()->pose.position.z);

  Eigen::Quaterniond normal_world(T_odom_body_.rotation() * T_body_cam_.rotation() * normal_cam);
  Eigen::Vector3d dist_body = T_body_cam_ * dist_cam;

  Eigen::Matrix3d normal_world_r = normal_world.toRotationMatrix();

  Eigen::Matrix3d new_orientation_world;
  Eigen::Vector3d gravity_W = -Eigen::Vector3d::UnitZ();

  // detect if normal and gravity are colinear
  if (gravity_W.cross(-normal_world_r.col(2)).norm() < 0.05) {
    new_orientation_world.col(1) << 0.0, 1.0, 0.0;  // align with y axis
    new_orientation_world.col(1).normalize();
    new_orientation_world.col(2) = -normal_world_r.col(2);
    new_orientation_world.col(2).normalize();
    new_orientation_world.col(0) = new_orientation_world.col(2).cross(new_orientation_world.col(1));
  } else {
    new_orientation_world.col(1) = normal_world_r.col(2).cross(gravity_W);
    new_orientation_world.col(1).normalize();
    new_orientation_world.col(2) = -normal_world_r.col(2);
    new_orientation_world.col(2).normalize();
    new_orientation_world.col(0) = new_orientation_world.col(2).cross(new_orientation_world.col(1));
  }

  // check righthanded
  if (new_orientation_world.determinant() < 0) {
    new_orientation_world.col(0) = -new_orientation_world.col(0);
  }

  Eigen::Quaterniond quat(new_orientation_world);
  desired_normal_orientation_ = quat;

  // calculate ??
  // double distance;
  // torolololo do pol_z_.setTarget()
}

void OmavPlanner::presenterCallback(const geometry_msgs::Vector3StampedPtr &vctr) {
  // set x/y targets in bodyframe



  double x = std::clamp(vctr->vector.x / 20.0, -20.0, 20.0);
  double y = std::clamp(vctr->vector.y / 20.0, -20.0, 20.0);

  if(abs(vctr->vector.x)<3 ){
    x= 0;
  }

  if(abs(vctr->vector.y)<3 ){
    y= 0;
  }

  ptr_target_ += Eigen::Vector3d(-y, x, 0.0);

  ptr_target_*=0.9;

  pol_body_ptr_->setTarget(ptr_target_);
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

  ros::Rate rate(100);  // ROS Rate at 5Hz
  while (ros::ok()) {
    ros::spinOnce();
    node.run();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
