#include "cpt_reconstruction/reconstruction_proposal_selection.h"

namespace cad_percept {
namespace cpt_reconstruction {
ProposalSelection::ProposalSelection(
    pcl::search::KdTree<pcl::PointXYZ>::Ptr upsampled_kd_tree,
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> meshing_clouds,
    std::vector<Eigen::Vector3d> &center_estimates,
    std::vector<Eigen::Matrix3d> &direction_estimates,
    std::vector<std::vector<Eigen::VectorXd>> &parameter_estimates,
    std::vector<Eigen::MatrixXd> &bounded_axis_estimates,
    std::vector<double> &radius_estimates)
    : model_upsampled_kdtree_(upsampled_kd_tree),
      scan_kdtree_(new pcl::search::KdTree<pcl::PointXYZ>()),
      center_estimates_(center_estimates),
      direction_estimates_(direction_estimates),
      parameter_estimates_(parameter_estimates),

      bounded_axis_estimates_(bounded_axis_estimates),
      radius_estimates_(radius_estimates) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr meshing_points (new pcl::PointCloud<pcl::PointXYZ>());
  for (int i = 0; i < meshing_clouds.size(); i++){
    (*meshing_points) += *(meshing_clouds.at(i));
  }
  scan_kdtree_->setInputCloud(meshing_points);
}

void ProposalSelection::selectProposals() {
  this->removeInsufficientElements();

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

  ROS_INFO("1");
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
  std::vector<std::vector<std::vector<double>>>
      parameters_in_buckets;  // Parameter estimates
  std::vector<std::vector<std::pair<int, int>>>
      id_parameters_in_buckets;  // Pair of (Element,Direction)

  for (int i = 0; i < dir_buckets.size(); i++) {
    std::vector<std::vector<double>> params_in_bucket;
    std::vector<std::pair<int, int>> id_params_in_bucket;

    Eigen::Vector3d dir_bucket = dir_buckets.at(i);

    for (int j = 0; j < bucket_assignments.size(); j++) {
      Eigen::Vector3d assigned_dirs = bucket_assignments.at(j);

      for (int k = 0; k < 3; k++) {
        if (assigned_dirs[k] == i) {
          std::vector<Eigen::VectorXd> magnitudes_element =
              parameter_estimates_.at(j);

          Eigen::VectorXd para_1;
          Eigen::VectorXd para_2;
          std::pair<int, int> id_1;
          std::pair<int, int> id_2;
          if (k == 0) {
            para_1 = magnitudes_element.at(0);
            para_2 = magnitudes_element.at(1);
            id_1 = std::make_pair(j, 0);
            id_2 = std::make_pair(j, 1);
          } else if (k == 1) {
            para_1 = magnitudes_element.at(2);
            para_2 = magnitudes_element.at(3);
            id_1 = std::make_pair(j, 2);
            id_2 = std::make_pair(j, 3);
          } else {
            para_1 = magnitudes_element.at(4);
            para_2 = magnitudes_element.at(5);
            id_1 = std::make_pair(j, 4);
            id_2 = std::make_pair(j, 5);
          }

          std::vector<double> params_per_element_1;
          std::vector<double> params_per_element_2;
          for (int l1 = 0; l1 < para_1.size(); l1++) {
            params_per_element_1.push_back(para_1[l1]);
          }
          for (int l2 = 0; l2 < para_2.size(); l2++) {
            params_per_element_2.push_back(para_2[l2]);
          }

          params_in_bucket.push_back(params_per_element_1);
          params_in_bucket.push_back(params_per_element_2);
          id_params_in_bucket.push_back(id_1);
          id_params_in_bucket.push_back(id_2);
        }
      }
    }
    parameters_in_buckets.push_back(params_in_bucket);
    id_parameters_in_buckets.push_back(id_params_in_bucket);
  }

  // Select combinations reducting the number of unique plans
  for (int i = 0; i < parameters_in_buckets.size(); i++) {
    std::vector<std::vector<double>> parameter_vec =
        parameters_in_buckets.at(i);
    std::vector<std::pair<int, int>> id_vec = id_parameters_in_buckets.at(i);
    if (parameter_vec.size() == 0) {
      continue;
    }

    // Compute meaningful possible combinations
    // Source:
    // https://stackoverflow.com/questions/48270565/create-all-possible-combinations-of-multiple-vectors
    while (parameter_vec.size() > 0) {
      std::vector<std::vector<double>> all_combination_values;
      std::vector<std::vector<std::pair<int, int>>>
          all_combination_ids;  // Element nr, and dir


      //Compute Range of d Parameters for first element
      Eigen::Vector3d center_0 = center_estimates_.at(id_vec.at(0).first);
      Eigen::Vector3d dir_0 = direction_estimates_.at(id_vec.at(0).first).col(id_vec.at(0).second % 3);
      double min_d0 = 10000;
      double max_d0 = -1000;
      for (auto const &cur_p : parameter_vec.at(0)){
        Eigen::Vector3d point_e = center_0 + dir_0 * cur_p;
        double cur_d = -(point_e.x() * dir_0.x() + point_e.y() * dir_0.y() + point_e.z() * dir_0.z());
        if (cur_d > max_d0){
          max_d0 =cur_d;
        }
        if (cur_d < min_d0){
          min_d0 =cur_d;
        }
      }

      //Search for max 2 elements with a d in the computed range
      std::vector<int> subset_idx;
      subset_idx.push_back(0);

      /*
      for (int p_idx = 1; p_idx < parameter_vec.size(); p_idx++){
        Eigen::Vector3d center_i = center_estimates_.at(id_vec.at(p_idx).first);
        Eigen::Vector3d dir_i = direction_estimates_.at(id_vec.at(p_idx).first).col(id_vec.at(p_idx).second % 3);
        for (auto const &param_i : parameter_vec.at(p_idx)){
          Eigen::Vector3d point_i = center_i + dir_i * param_i;
          double cur_d = -(point_i.x() * dir_i.x() + point_i.y() * dir_i.y() + point_i.z() * dir_i.z());
          if (cur_d > min_d0 && cur_d < max_d0){
            subset_idx.push_back(p_idx);
            break;
          }
        }
        if (subset_idx.size() > 1){
          break;
        }
      }
      */
      std::vector<std::vector<double>> parameter_vec_subset;
      std::vector<std::pair<int, int>> id_vec_subset;
      for (auto const &idx_to_add : subset_idx){
        parameter_vec_subset.push_back(parameter_vec.at(idx_to_add));
        id_vec_subset.push_back(id_vec.at(idx_to_add));
      }

      int idx_corr = 0;
      for (unsigned r = 0; r < subset_idx.size(); r++) {
        int idx = subset_idx.at(r);
        parameter_vec.erase(parameter_vec.begin() + idx - idx_corr);
        id_vec.erase(id_vec.begin() + idx - idx_corr);
        idx_corr++;
      }

      unsigned long long int max = 1;
      for (auto const &v : parameter_vec_subset) {
        max *= v.size();
      }
      ROS_INFO("Number of Combinations %d", max);

      for (unsigned long long int j = 0; j < max; j++) {
        auto temp = j;
        std::vector<double> cur_combination_values;
        std::vector<std::pair<int, int>> cur_combination_ids;
        for (int k = 0; k < parameter_vec_subset.size(); k++) {
          std::vector<double> cur_vec = parameter_vec_subset.at(k);
          std::pair<int, int> cur_ids = id_vec_subset.at(k);
          auto index = temp % cur_vec.size();
          temp /= cur_vec.size();
          cur_combination_values.push_back(cur_vec[index]);
          cur_combination_ids.push_back(cur_ids);
        }
        all_combination_values.push_back(cur_combination_values);
        all_combination_ids.push_back(cur_combination_ids);
      }

      // Evaluate combinations
      std::vector<double> combination_scores;
      for (int l = 0; l < all_combination_values.size(); l++) {
        std::vector<double> cur_combination = all_combination_values.at(l);
        std::vector<std::pair<int, int>> cur_idx = all_combination_ids.at(l);
        // Area value
        double area_score = 0;
        for (auto &c : cur_combination) {
          area_score += std::fabs(c);
        }

        //Score for model alignment
        double model_alignment_score = 0;
        double scan_alignment_score = 0;
        // Compute d parameter and model alignment score
        std::vector<double> d_parameters;
        std::vector<int> nn_indices{1};
        std::vector<float> nn_dists{1};
        for (int d_o = 0; d_o < cur_combination.size(); d_o ++){
          Eigen::Vector3d center_i = center_estimates_.at(cur_idx.at(d_o).first);
          Eigen::Vector3d dir_i = direction_estimates_.at(cur_idx.at(d_o).first).col(cur_idx.at(d_o).second % 3);
          Eigen::Vector3d point_i = center_i + dir_i * cur_combination.at(d_o);
          pcl::PointXYZ point_pcl (point_i.x(), point_i.y(), point_i.z());
          model_upsampled_kdtree_->nearestKSearch(point_pcl, 1, nn_indices, nn_dists);
          if (std::sqrt(nn_dists[0]) < 0.03){
            model_alignment_score += 1.0 - 33.3 * std::sqrt(nn_dists[0]);
          }
          scan_kdtree_->nearestKSearch(point_pcl, 1, nn_indices, nn_dists);
          if (std::sqrt(nn_dists[0]) < 0.03){
            scan_alignment_score += 1.0 - 33.3 * std::sqrt(nn_dists[0]);
          }

          double cur_d = -(point_i.x() * dir_i.x() + point_i.y() * dir_i.y() + point_i.z() * dir_i.z());
          d_parameters.push_back(cur_d);
        }
        removeDuplicatedValues(d_parameters, 0.03);

        int shared_planes = cur_combination.size() - d_parameters.size();

        double total_score = 2.0 * area_score + 0.0 * shared_planes + 5.0 * model_alignment_score + 0.0 * scan_alignment_score;
        combination_scores.push_back(total_score);
      }
      int max_idx = std::max_element(combination_scores.begin(),
                                     combination_scores.end()) -
                    combination_scores.begin();
      std::vector<double> final_combination =
          all_combination_values.at(max_idx);
      std::vector<std::pair<int, int>> final_idx =
          all_combination_ids.at(max_idx);
      for (int f = 0; f < final_combination.size(); f++) {
        std::pair<int, int> f_id = final_idx.at(f);
        parameter_estimates_.at(f_id.first).at(f_id.second).resize(1, 1);
        parameter_estimates_.at(f_id.first).at(f_id.second)[0] =
            final_combination.at(f);
      }
    }
  }
  //this->organizeDatastructure();
  //this->removeConflictingElements();
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /*
  // Get Min and Max parameter and setup a grid structure
  double min_parameter = 1000;
  double max_parameter = -1000;
  for (int j = 0; j < parameter_vec.size(); j++) {
    std::vector<double> cur_element_dim = parameter_vec.at(j);
    for (int k = 0; k < cur_element_dim.size(); k++) {
      double target = cur_element_dim.at(k);
      if (target > max_parameter) {
        max_parameter = target;
      }
      if (target < min_parameter) {
        min_parameter = target;
      }
    }
  }
  int length = (int)((max_parameter - min_parameter) / 0.03) + 10;
  std::vector<int> plane_counter(length, 0);

  // Count planes
  for (int j = 0; j < parameter_vec.size(); j++) {
    std::vector<double> cur_element_dim = parameter_vec.at(j);
    for (int k = 0; k < cur_element_dim.size(); k++) {
      double target = cur_element_dim.at(k);
      int idx = (int)std::round((target - min_parameter) / 0.03);
      plane_counter.at(idx) += 1;
    }
  }
  int max_plane_counter =
      *std::max_element(plane_counter.begin(), plane_counter.end());

  for (int j = 0; j < parameter_vec.size(); j++) {
    std::vector<double> cur_element_dim = parameter_vec.at(j);
    std::vector<double> score_cur_element_dim = score_parameter_vec.at(j);
    for (int k = 0; k < cur_element_dim.size(); k++) {
      double target = cur_element_dim.at(k);
      int idx = (int)std::round((target - min_parameter) / 0.03);
      score_parameters_in_buckets.at(i).at(j).at(k) *=
          ((double)plane_counter.at(idx) / (double)max_plane_counter);
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

// Select most likely Parameter
for (int i = 0; i < score_parameters_in_buckets.size(); i++) {
  for (int j = 0; j < score_parameters_in_buckets.at(i).size(); j++) {
    std::vector<double> cur_scores = score_parameters_in_buckets.at(i).at(j);
    std::pair<int, int> cur_id = id_parameters_in_buckets.at(i).at(j);

    int element_nr = cur_id.first;
    int parameter_nr = cur_id.second;

    int max_idx = std::max_element(cur_scores.begin(), cur_scores.end()) -
                  cur_scores.begin();
    double most_likely_parameter =
        parameter_estimates_.at(element_nr).at(parameter_nr)[max_idx];
    parameter_estimates_.at(element_nr).at(parameter_nr).resize(1, 1);
    parameter_estimates_.at(element_nr).at(parameter_nr)[0] =
        most_likely_parameter;
  }
}
*/
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

    int counter_non_zero_elements = 0;
    if (a1.size() > 1){
      counter_non_zero_elements++;
    }
    if (a2.size() > 1){
      counter_non_zero_elements++;
    }
    if (b1.size() > 1){
      counter_non_zero_elements++;
    }
    if (b2.size() > 1){
      counter_non_zero_elements++;
    }
    if (c1.size() > 1){
      counter_non_zero_elements++;
    }
    if (c2.size() > 1){
      counter_non_zero_elements++;
    }

    //Allow for one missed parameter
    if (counter_non_zero_elements <= 4){
      elements_to_remove.push_back(i);
      ROS_INFO("Removed element due to insufficent number of paramters");
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

void ProposalSelection::getSelectedProposals(
    std::vector<Eigen::Vector3d> &center_estimates,
    std::vector<Eigen::Matrix3d> &direction_estimates,
    std::vector<std::vector<Eigen::VectorXd>> &parameter_estimates) {
  center_estimates.clear();
  direction_estimates.clear();
  parameter_estimates.clear();
  center_estimates = center_estimates_;
  direction_estimates = direction_estimates_;
  parameter_estimates = parameter_estimates_;
}

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

    double a_max = a1[0];
    double a_min = a2[0];
    double b_max = b1[0];
    double b_min = b2[0];
    double c_max = c1[0];
    double c_min = c2[0];

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

void ProposalSelection::removeDuplicatedValues(std::vector<double> &vector,
                                            double eps) {
  // Source:
  // https://stackoverflow.com/questions/34481190/removing-duplicates-of-3d-points-in-a-vector-in-c/34481426
  std::sort(vector.begin(), vector.end(),
            [eps](double p1, double p2) -> bool { return p1 > p2; });
  auto unique_end = std::unique(vector.begin(), vector.end(),
                                [eps](double p1, double p2) -> bool {
                                  if (std::fabs(p1 - p2) < eps) {
                                    return true;
                                  }
                                  return false;
                                });
  vector.erase(unique_end, vector.end());
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
    if (coverage > 0.75) {
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
