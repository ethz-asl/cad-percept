#include <cgal_conversions/eigen_conversions.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <kindr/minimal/quat-transformation-gtsam.h>

#include "cpt_localization/mesh_localizer.h"

namespace cad_percept {
namespace localization {

MeshLocalizer::MeshLocalizer() {}

MeshLocalizer::MeshLocalizer(const std::string &model_file) : mesh_model_
                                                                  (model_file) {}

MeshLocalizer::MeshLocalizer(const cgal::Polyhedron &mesh) {
  setMesh(mesh);
}

MeshLocalizer::~MeshLocalizer() {}

Associations MeshLocalizer::associatePointCloud(const PointCloud &pc_msg) const {
  std::cout << "Associating pointcloud of size " << pc_msg.width << " "
            << pc_msg.height << std::endl;
  Associations associations;
  associations.points_from.resize(3, pc_msg.width);
  associations.points_to.resize(3, pc_msg.width);
  associations.normals_to.resize(3, pc_msg.width);
  associations.distances.resize(pc_msg.width);
  for (size_t i = 0u; i < pc_msg.width; ++i) {

    cgal::PointAndPrimitiveId ppid =
        mesh_model_.getClosestTriangle(pc_msg[i].x,
                                       pc_msg[i].y,
                                       pc_msg[i].z);
    cgal::Point pt = ppid.first;
    associations.points_from(0, i) = pc_msg[i].x;
    associations.points_from(1, i) = pc_msg[i].y;
    associations.points_from(2, i) = pc_msg[i].z;

    // Raycast into direction of triangle normal.
    Eigen::Vector3d normal;
    cgal::cgalPointToEigenVector(mesh_model_.getNormal(ppid), &normal);
    normal.normalize();
    associations.normals_to.block(0, i, 3, 1) = normal;
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

SE3 MeshLocalizer::icm(const PointCloud &pc_msg, const SE3 &initial_pose) {
  bool iterate = true;
  PointCloud point_cloud = pc_msg;
  gtsam::noiseModel::Base::shared_ptr match_noise;
  Eigen::Matrix<double, 1, 1> noise_model;
  noise_model = Eigen::Matrix<double, 1, 1>::Ones() * 0.02;

  match_noise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Cauchy::Create(1),
      gtsam::noiseModel::Diagonal::Sigmas(noise_model));

  // todo: should we iterate and reassociate?

  // Associate point cloud with mesh.
  Associations associations = associatePointCloud(point_cloud);
  // todo: Weight outliers? Or simply use robust cost function?

  SE3 T_W_A, T_W_B;
  T_W_A = SE3(SE3::Position(0, 0, 0), SE3::Rotation(1, 0, 0, 0));
  T_W_B = initial_pose;

  gtsam::Expression<SE3> E_T_W_A(0); // 0,0,0,0 (origin).
  gtsam::Expression<SE3> E_T_W_B(1); // Current pose.

  // Anchor map pose with prior factor.
  Eigen::Matrix<double, 6, 1> noise;
  noise(0) = 0.0000001;
  noise(1) = 0.0000001;
  noise(2) = 0.0000001;
  noise(3) = 0.0000001;
  noise(4) = 0.0000001;
  noise(5) = 0.0000001;

  gtsam::noiseModel::Diagonal::shared_ptr prior_noise =
      gtsam::noiseModel::Diagonal::Sigmas(noise);
  gtsam::ExpressionFactor<SE3> anchor(prior_noise, T_W_A,
                                      E_T_W_A);
  factor_graph_.push_back(anchor);

  for (size_t i = 0u; i < associations.points_from.cols(); ++i) {
    // Reduce error with GTSAM.
    // Create expression factors from associations and add to factor graph.
    SE3::Position mu_W_SA = associations.points_to.block(0, i, 3, 1);
    SE3::Position mu_W_SB = associations.points_from.block(0, i, 3, 1);
    SE3::Position mu_A_SA, mu_B_SB; // The individual points.
    mu_A_SA = T_W_A.inverse().transform(mu_W_SA);
    mu_B_SB = T_W_B.inverse().transform(mu_W_SB);
    SE3::Position normalRef_l = T_W_A.getRotation().inverseRotate
        (associations.normals_to.block(0, i, 3, 1));
    gtsam::Expression<Eigen::Vector3d> E_normalRef_l(normalRef_l);
    gtsam::Expression<Eigen::Vector3d> E_normalRef_w =
        kindr::minimal::rotate(kindr::minimal::rotationFromTransformation(
            E_T_W_A), E_normalRef_l);

    gtsam::Expression<Eigen::Vector3d> E_mu_A_SA(mu_A_SA);
    gtsam::Expression<Eigen::Vector3d> E_mu_B_SB(mu_B_SB);

    gtsam::Expression<Eigen::Vector3d>
        pointTransformedB = kindr::minimal::transform(E_T_W_B, E_mu_B_SB);
    gtsam::Expression<Eigen::Vector3d>
        pointTransformedA = kindr::minimal::transform(E_T_W_A, E_mu_A_SA);
    gtsam::Expression<Eigen::Vector3d> subtracted =
        kindr::minimal::vectorDifference(pointTransformedA,
                                         pointTransformedB);
    gtsam::Expression<double>
        error = multiplyVectors(subtracted, E_normalRef_w);
    gtsam::ExpressionFactor<double> match_factor(match_noise, (double(0)),
                                                 error);
    factor_graph_.push_back(match_factor);
  }
  gtsam::Values initial_estimate;
  initial_estimate.insert(0, T_W_A);
  initial_estimate.insert(1, T_W_B);
  gtsam::LevenbergMarquardtParams params;
  params.setVerbosity("LINEAR"); // LINEAR - ERROR
//    params.setMaxIterations(2);
  gtsam::LevenbergMarquardtOptimizer optimizer(factor_graph_,
                                               initial_estimate, params);
  gtsam::Values result = optimizer.optimize();
  result.print("Result from optimization");
  // todo: Update point_cloud with transformation.
  return result.at<SE3>(1);
}

void MeshLocalizer::transformModel(const Eigen::Matrix4d &transformation) {
  cgal::Transformation
      trans(transformation(0, 0),
            transformation(0, 1),
            transformation(0, 2),
            transformation(0, 3),
            transformation(1, 0),
            transformation(1, 1),
            transformation(1, 2),
            transformation(1, 3),
            transformation(2, 0),
            transformation(2, 1),
            transformation(2, 2),
            transformation(2, 3),
            1.0);
  mesh_model_.transform(trans);
}

void MeshLocalizer::setMesh(const cgal::Polyhedron &mesh) {
  mesh_model_.setSurfaceMesh(mesh);
}

}
}