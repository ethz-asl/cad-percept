#include "cpt_reconstruction/reconstruction_proposal_selection.h"

namespace cad_percept {
namespace cpt_reconstruction {
ProposalSelection::ProposalSelection(
    std::vector<Eigen::Vector3d> &center_estimates,
    std::vector<Eigen::Matrix3d> &direction_estimates,
    std::vector<std::vector<Eigen::VectorXd>> &parameter_estimates,
    std::vector<Eigen::MatrixXd> &bounded_axis_estimates,
    std::vector<double> &radius_estimates)
    : center_estimates_(center_estimates),
      direction_estimates_(direction_estimates),
      parameter_estimates_(parameter_estimates),

      bounded_axis_estimates_(bounded_axis_estimates),
      radius_estimates_(radius_estimates) {}

void ProposalSelection::selectProposals() {
  this->removeInsufficientElements();
  this->organizeDatastructure();
  this->removeConflictingElements();

  // Minimize Number of unique Plane, Favour Model Structure and Size

  std::vector<Eigen::Vector3d> dir_buckets;
  // Z = 0
  dir_buckets.push_back(Eigen::Vector3d(1, 0, 0));
  dir_buckets.push_back(Eigen::Vector3d(0, 1, 0));
  dir_buckets.push_back(Eigen::Vector3d(0.7071, 0.7071, 0));
  dir_buckets.push_back(Eigen::Vector3d(0.7071, -0.7071, 0));

  // Z = - 1
  dir_buckets.push_back(Eigen::Vector3d(0.7071, 0, 0.7071));
  dir_buckets.push_back(Eigen::Vector3d(0, 0.7071, 0.7071));
  dir_buckets.push_back(Eigen::Vector3d(0.5774, 0.5774, 0.5774));
  dir_buckets.push_back(Eigen::Vector3d(0.5774, -0.5774, 0.5774));

  // Z = 1
  dir_buckets.push_back(Eigen::Vector3d(0.7071, 0, -0.7071));
  dir_buckets.push_back(Eigen::Vector3d(0, 0.7071, -0.7071));
  dir_buckets.push_back(Eigen::Vector3d(0.5774, 0.5774, -0.5774));
  dir_buckets.push_back(Eigen::Vector3d(0.5774, -0.5774, -0.5774));
  dir_buckets.push_back(Eigen::Vector3d(0, 0, 1));

  std::vector<Eigen::Vector3d> bucket_assignments;
  for (int i = 0; i < direction_estimates_.size(); i++) {
    Eigen::Matrix3d directions = direction_estimates_.at(i);
    Eigen::Vector3d dir_1 = directions.col(0);
    Eigen::Vector3d dir_2 = directions.col(1);
    Eigen::Vector3d dir_3 = directions.col(2);

    Eigen::Vector3d assignment;
    double score_1 = -1.0;
    double score_2 = -1.0;
    double score_3 = -1.0;
    for (int j = 0; j < dir_buckets.size(); j++) {
      Eigen::Vector3d cur_bucket = dir_buckets.at(j);

      double dot_1 = std::fabs(cur_bucket.dot(dir_1));
      double dot_2 = std::fabs(cur_bucket.dot(dir_2));
      double dot_3 = std::fabs(cur_bucket.dot(dir_3));

      if (dot_1 > score_1) {
        score_1 = dot_1;
        assignment.x() = j;
      }
      if (dot_2 > score_2) {
        score_2 = dot_2;
        assignment.y() = j;
      }
      if (dot_3 > score_3) {
        score_3 = dot_3;
        assignment.z() = j;
      }
    }
    bucket_assignments.push_back(assignment);
  }

  // Evaluate plane coeffs
  std::vector<std::vector<std::vector<double>>> parameters_in_buckets;
  std::vector<std::vector<std::vector<double>>> score_parameters_in_buckets;
  for (int i = 0; i < dir_buckets.size(); i++) {
    std::vector<std::vector<double>> params_in_bucket;
    std::vector<std::vector<double>> score_params_in_bucket;
    Eigen::Vector3d dir_bucket = dir_buckets.at(i);

    for (int j = 0; j < bucket_assignments.size(); j++) {
      Eigen::Vector3d assigned_dirs = bucket_assignments.at(j);

      for (int k = 0; k < 3; k++) {
        if (assigned_dirs[k] == i) {
          Eigen::Vector3d center_of_element = center_estimates_.at(j);
          Eigen::Matrix3d dir_element = direction_estimates_.at(j);
          std::vector<Eigen::VectorXd> magnitudes_element =
              parameter_estimates_.at(j);

          Eigen::VectorXd para_1 = magnitudes_element.at(k);
          Eigen::VectorXd para_2 = magnitudes_element.at(k + 3);

          std::vector<double> params_per_element_1;
          std::vector<double> params_per_element_2;

          std::vector<double> score_params_per_element_1;
          std::vector<double> score_params_per_element_2;

          double norm_para_1 = para_1.norm();
          double norm_para_2 = para_2.norm();

          for (int l = 0; l < para_1.size(); l++) {
            Eigen::Vector3d point =
                center_of_element + para_1[l] * dir_element.col(k);
            double d =
                -(point.x() * dir_bucket.x() + point.y() * dir_bucket.y() +
                  point.z() * dir_bucket.z());
            params_per_element_1.push_back(d);
            score_params_per_element_1.push_back(
                std::fabs(para_1[l] / norm_para_1));
          }
          for (int m = 0; m < para_2.size(); m++) {
            Eigen::Vector3d point =
                center_of_element + para_2[m] * dir_element.col(k);
            double d =
                -(point.x() * dir_bucket.x() + point.y() * dir_bucket.y() +
                  point.z() * dir_bucket.z());
            params_per_element_2.push_back(d);
            score_params_per_element_2.push_back(
                std::fabs(para_2[m] / norm_para_2));
          }
          params_in_bucket.push_back(params_per_element_1);
          params_in_bucket.push_back(params_per_element_2);
          score_params_in_bucket.push_back(score_params_per_element_1);
          score_params_in_bucket.push_back(score_params_per_element_2);
        }
      }
    }
    parameters_in_buckets.push_back(params_in_bucket);
    score_parameters_in_buckets.push_back(score_params_in_bucket);
  }

  // Select combinations reducting the number of unique plans
  for (int i = 0; i < parameters_in_buckets.size(); i++) {
    std::vector<std::vector<double>> parameter_vec =
        parameters_in_buckets.at(i);
    std::vector<std::vector<double>> score_parameter_vec =
        score_parameters_in_buckets.at(i);

    for (int j = 0; j < parameter_vec.size(); j++) {
      std::vector<double> cur_element_dim = parameter_vec.at(j);
      std::vector<double> score_cur_element_dim = score_parameter_vec.at(j);
      for (int k = 0; k < cur_element_dim.size(); k++) {
        double target = cur_element_dim.at(k);

        double similarity_score = 0;
        for (int l1 = 0; l1 < parameter_vec.size(); l1++) {
          if (l1 == j) {
            continue;
          }
          for (int l2 = 0; l2 < parameter_vec.at(l1).size(); l2++) {
            double candidate = parameter_vec.at(l1).at(l2);
            double diff = std::fabs(candidate - target);
            if (diff < 0.1) {
              similarity_score += (0.05 / (diff + 10e-8));
            }
          }
        }
        score_parameters_in_buckets.at(i).at(j).at(k) += similarity_score;
      }
    }
  }

  // TODO Remove
  for (int i = 0; i < parameters_in_buckets.size(); i++) {
    ROS_INFO("BUCKET_NUMBER: %d", i);
    std::vector<std::vector<double>> parameter_vec =
        parameters_in_buckets.at(i);
    std::vector<std::vector<double>> score_parameter_vec =
        score_parameters_in_buckets.at(i);
    for (int j = 0; j < parameter_vec.size(); j++) {
      ROS_INFO("  Parameters Per Element: %d", j);
      std::vector<double> cur_element_dim = parameter_vec.at(j);
      std::vector<double> score_cur_element_dim = score_parameter_vec.at(j);
      for (int k = 0; k < cur_element_dim.size(); k++) {
        ROS_INFO("      Para: %f, Score: %f", cur_element_dim.at(k),
                 score_cur_element_dim.at(k));
      }
    }
  }
}

void ProposalSelection::removeInsufficientElements() {
  std::vector<int> elements_to_remove;
  for (int i = 0; i < parameter_estimates_.size(); i++) {
    std::vector<Eigen::VectorXd> magnitudes = parameter_estimates_.at(i);

    Eigen::VectorXd a1 = magnitudes.at(0);
    Eigen::VectorXd a2 = magnitudes.at(1);
    Eigen::VectorXd b1 = magnitudes.at(2);
    Eigen::VectorXd b2 = magnitudes.at(3);
    Eigen::VectorXd c1 = magnitudes.at(4);
    Eigen::VectorXd c2 = magnitudes.at(5);

    if (a1.size() == 0 || a2.size() == 0 || b1.size() == 0 || b2.size() == 0 ||
        c1.size() == 0 || c2.size() == 0) {
      elements_to_remove.push_back(i);
    }
  }
  int idx_corr = 0;
  for (unsigned r = 0; r < elements_to_remove.size(); r++) {
    int idx = elements_to_remove.at(r);
    center_estimates_.erase(center_estimates_.begin() + idx - idx_corr);
    direction_estimates_.erase(direction_estimates_.begin() + idx - idx_corr);
    parameter_estimates_.erase(parameter_estimates_.begin() + idx - idx_corr);
    idx_corr++;
  }
}

void ProposalSelection::getSelectedProposals() {}

void ProposalSelection::organizeDatastructure() {
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
    int steps_a = (int)((a_max - a_min) / step + 1);
    int steps_b = (int)((b_max - b_min) / step + 1);
    int steps_c = (int)((c_max - c_min) / step + 1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cur_cloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    for (int i1 = 0; i1 < steps_a; i1++) {
      double cur_a = (a_min + step * i1);
      for (int i2 = 0; i2 < steps_b; i2++) {
        double cur_b = (b_min + step * i2);
        for (int i3 = 0; i3 < steps_c; i3++) {
          double cur_c = (c_min + step * i3);

          Eigen::Vector3d cur_e = center + directions.col(0) * cur_a +
                                  directions.col(1) * cur_b +
                                  directions.col(2) * cur_c;

          pcl::PointXYZ p((float)cur_e.x(), (float)cur_e.y(), (float)cur_e.z());
          cur_cloud->push_back(p);
        }
      }
    }

    pcl::search::KdTree<pcl::PointXYZ>::Ptr cur_kd_tree(
        new pcl::search::KdTree<pcl::PointXYZ>());
    cur_kd_tree->setInputCloud(cur_cloud);

    structured_point_clouds_.push_back(cur_cloud);
    kd_trees_.push_back(cur_kd_tree);
  }
}

void ProposalSelection::removeConflictingElements() {
  int number_planar_elements = structured_point_clouds_.size();
  std::vector<int> nn_indices{1};
  std::vector<float> nn_dists{1};

  std::vector<int> conflicting_elements;
  for (int i = 0; i < number_planar_elements; i++) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cur_cloud =
        structured_point_clouds_.at(i);

    int nr_matches = 0;
    for (int j = 0; j < cur_cloud->size(); j++) {
      pcl::PointXYZ p = (*cur_cloud)[j];
      for (int k = 0; k < number_planar_elements; k++) {
        if (std::find(conflicting_elements.begin(), conflicting_elements.end(),
                      k) != conflicting_elements.end() ||
            i == k) {
          continue;
        }

        pcl::search::KdTree<pcl::PointXYZ>::Ptr cur_kd_tree = kd_trees_.at(k);
        cur_kd_tree->nearestKSearch(p, 1, nn_indices, nn_dists);
        if (std::sqrt(nn_dists[0]) < 0.05) {
          nr_matches++;
          break;
        }
      }
    }
    double coverage = ((double)nr_matches / (double)cur_cloud->size());
    if (coverage > 0.8) {
      conflicting_elements.push_back(i);
    }
  }

  // TODO Remove
  pcl::PointCloud<pcl::PointXYZ>::Ptr valid_elements(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr invalid_elements(
      new pcl::PointCloud<pcl::PointXYZ>());
  for (int i = 0; i < number_planar_elements; i++) {
    if (std::find(conflicting_elements.begin(), conflicting_elements.end(),
                  i) != conflicting_elements.end()) {
      (*invalid_elements) += *(structured_point_clouds_.at(i));
    } else {
      (*valid_elements) += *(structured_point_clouds_.at(i));
    }
  }
  pcl::io::savePLYFile("/home/philipp/Schreibtisch/ros_dir/valid_elements.ply",
                       *valid_elements);
  pcl::io::savePLYFile(
      "/home/philipp/Schreibtisch/ros_dir/invalid_elements.ply",
      *invalid_elements);

  int idx_corr = 0;
  for (unsigned r = 0; r < conflicting_elements.size(); r++) {
    int idx = conflicting_elements.at(r);
    center_estimates_.erase(center_estimates_.begin() + idx - idx_corr);
    direction_estimates_.erase(direction_estimates_.begin() + idx - idx_corr);
    parameter_estimates_.erase(parameter_estimates_.begin() + idx - idx_corr);
    idx_corr++;
  }
}

}  // namespace cpt_reconstruction
}  // namespace cad_percept
