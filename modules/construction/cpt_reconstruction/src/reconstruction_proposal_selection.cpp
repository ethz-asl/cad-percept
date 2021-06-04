#include "cpt_reconstruction/reconstruction_proposal_selection.h"

namespace cad_percept {
namespace cpt_reconstruction {
ProposalSelection::ProposalSelection(std::vector<Eigen::Vector3d> &center_estimates,
                                     std::vector<Eigen::Matrix3d> &direction_estimates,
                                     std::vector<std::vector<Eigen::VectorXd>> &parameter_estimates,
                                     std::vector<Eigen::MatrixXd> &bounded_axis_estimates,
                                     std::vector<double> &radius_estimates) :
    center_estimates_(center_estimates),
    direction_estimates_(direction_estimates),
    parameter_estimates_(parameter_estimates),

    bounded_axis_estimates_(bounded_axis_estimates),
    radius_estimates_(radius_estimates) {
}

void ProposalSelection::selectProposals() {

  this->organizeDatastructure();

  // Minimize Number of unique Plane, Favour Model Structure and Size


}

void ProposalSelection::removeConflictingElements(){
  int nr_planar_elements =


}

void ProposalSelection::organizeDatastructure(){
  int nr_planar_elements = center_estimates_.size();
  int nr_cylinder_elements = radius_estimates_.size();

  for (int i = 0; i < nr_planar_elements; i++) {
    Eigen::Vector3d center = center_estimates_.at(i);
    Eigen::MatrixXd directions = direction_estimates_.at(i);
    std::vector<Eigen::VectorXd> magnitudes = parameter_estimates_.at(i);

    Eigen::VectorXd a1 = magnitudes.at(0);
    Eigen::VectorXd a2 = magnitudes.at(1);
    Eigen::VectorXd b1 = magnitudes.at(2);
    Eigen::VectorXd b2 = magnitudes.at(3);
    Eigen::VectorXd c1 = magnitudes.at(4);
    Eigen::VectorXd c2 = magnitudes.at(5);

    if (a1.size() > 0 && a2.size() > 0 &&
        b1.size() > 0 && b2.size() > 0 &&
        c1.size() > 0 && c2.size() > 0) {
      double a_max = a1.maxCoeff();
      double a_min = a2.minCoeff();
      double b_max = b1.maxCoeff();
      double b_min = b2.minCoeff();
      double c_max = c1.maxCoeff();
      double c_min = c2.minCoeff();

      ROS_INFO("a_max: %f", a_max);
      ROS_INFO("a_min: %f", a_min);
      ROS_INFO("b_max: %f", b_max);
      ROS_INFO("b_min: %f", b_min);
      ROS_INFO("c_max: %f", c_max);
      ROS_INFO("c_min: %f", c_min);

      double step = 0.05;
      int steps_a = (int) ((a_max - a_min) / step + 1);
      int steps_b = (int) ((b_max - b_min) / step + 1);
      int steps_c = (int) ((c_max - c_min) / step + 1);

      pcl::PointCloud<pcl::PointXYZ>::Ptr cur_cloud(new pcl::PointCloud<pcl::PointXYZ>());
      for (int i1 = 0; i1 < steps_a; i1++) {
        double cur_a = (a_min + step * i1);
        for (int i2 = 0; i2 < steps_b; i2++) {
          double cur_b = (b_min + step * i2);
          for (int i3 = 0; i3 < steps_c; i3++) {
            double cur_c = (c_min + step * i3);

            Eigen::Vector3d cur_e = center + directions.col(0) * cur_a +
                directions.col(1) * cur_b + directions.col(2) * cur_c;

            pcl::PointXYZ p((float) cur_e.x(), (float) cur_e.y(), (float) cur_e.z());
            cur_cloud->push_back(p);
          }
        }
      }

      pcl::search::KdTree<pcl::PointXYZ>::Ptr cur_kd_tree(new pcl::search::KdTree<pcl::PointXYZ>());
      cur_kd_tree->setInputCloud(cur_cloud);

      structured_point_clouds_.push_back(cur_cloud);
      kd_trees_.push_back(cur_kd_tree);
    }
  }
}

void ProposalSelection::getSelectedProposals() {


}

}
}
