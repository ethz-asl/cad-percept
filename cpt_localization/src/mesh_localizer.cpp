#include <kindr/minimal/quat-transformation-gtsam.h>

#include "cpt_localization/mesh_localizer.h"

namespace cad_percept {
namespace localization {

Associations MeshLocalizer::associatePointCloud(const PointCloud &pc_msg) const {
  std::cout << "Associating pointcloud of size " << pc_msg.width << " "
            << pc_msg.height << std::endl;
  Associations associations;
  associations.points_from.resize(3, pc_msg.width);
  associations.points_to.resize(3, pc_msg.width);
  associations.distances.resize(pc_msg.width);
  for (size_t i = 0u; i < pc_msg.width; ++i) {

    cgal::PointAndPrimitiveId ppid =
        mesh_model_->getClosestTriangle(pc_msg[i].x,
                                        pc_msg[i].y,
                                        pc_msg[i].z);
    cgal::Point pt = ppid.first;
    associations.points_from(0, i) = pc_msg[i].x;
    associations.points_from(1, i) = pc_msg[i].y;
    associations.points_from(2, i) = pc_msg[i].z;

    // Raycast into direction of triangle normal.
    Eigen::Vector3d normal = mesh_model_->getNormal(ppid);
    normal.normalize();
    Eigen::Vector3d relative = Eigen::Vector3d(pt.x(), pt.y(), pt.z())
        - Eigen::Vector3d(pc_msg[i].x, pc_msg[i].y, pc_msg[i].z);
    Eigen::Vector3d direction = normal.dot(relative) * normal;

    associations.points_to(0, i) =
        associations.points_from(0, i) + direction(0);
    associations.points_to(1, i) =
        associations.points_from(1, i) + direction(1);
    associations.points_to(2, i) =
        associations.points_from(2, i) + direction(2);
    associations.distances(i) = direction.norm();
  }
  return associations;
}

void MeshLocalizer::icm(const PointCloud &pc_msg, const SE3 &initial_pose) {
  bool iterate = true;
  PointCloud point_cloud = pc_msg;
  SE3 pose = initial_pose;
  gtsam::noiseModel::Base::shared_ptr match_noise;
  Eigen::Matrix<double,6,1> noise_model;
  noise_model = Eigen::Matrix<double, 6, 1>::Ones() * 0.1;
  match_noise = gtsam::noiseModel::Diagonal::Sigmas(noise_model);
  while (iterate) {
    // Associate point cloud with mesh.
    Associations associations = associatePointCloud(point_cloud);
    // Weight outliers? Or simply use robust cost function?

    // Reduce error with GTSAM?.
    // Create expression factors from associations and add to factor graph.
    SE3::Position mu_A_SA, mu_B_SB; // The point measures in ref frames.
    Eigen::Vector3d normal_SA;
    SE3::Position normalRef_l = pose.getRotation().inverseRotate
        (normal_SA);
    gtsam::Expression<Eigen::Vector3d> E_normalRef_l(normalRef_l);
    gtsam::Expression<Eigen::Vector3d> E_mu_A_SA(mu_A_SA);
    gtsam::Expression<Eigen::Vector3d> E_mu_B_SB(mu_B_SB);
    gtsam::Expression<SE3> T_W_A(pose); // 0,0,0,0 (origin).
    gtsam::Expression<SE3> T_W_B(pose); // Current pose.
    gtsam::Expression<Eigen::Vector3d> pointTransformedB = kindr::minimal::transform(T_W_B, E_mu_B_SB);
    gtsam::Expression<Eigen::Vector3d> pointTransformedA = kindr::minimal::transform(T_W_A, E_mu_A_SA);
    gtsam::Expression<Eigen::Vector3d> substracted = kindr::minimal::vectorDifference(pointTransformedA, pointTransformedB);
    gtsam::Expression<Eigen::Vector3d> E_normalRef_w = kindr::minimal::rotate(kindr::minimal::rotationFromTransformation(T_W_A),
                                                                       E_normalRef_l);
    gtsam::Expression<double> error = multiplyVectors(substracted,E_normalRef_w);
    gtsam::ExpressionFactor<double> match_factor(match_noise,(double (0)),
        error);
    factor_graph_.push_back(match_factor);







    // Update point_cloud with transformation.

    // Add stop condition.
    iterate = false;
  }
}

}
}