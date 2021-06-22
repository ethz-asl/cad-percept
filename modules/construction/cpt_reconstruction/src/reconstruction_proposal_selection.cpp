#include "cpt_reconstruction/reconstruction_proposal_selection.h"

namespace cad_percept {
namespace cpt_reconstruction {
ProposalSelection::ProposalSelection(
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr upsampled_octree,
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> meshing_clouds,
    pcl::PolygonMesh mesh_model,
    pcl::PointCloud<pcl::PointXYZ>::Ptr model_upsampled_points,
    std::vector<Eigen::Vector3d> &center_estimates,
    std::vector<Eigen::Matrix3d> &direction_estimates,
    std::vector<std::vector<Eigen::VectorXd>> &parameter_estimates,
    std::vector<Eigen::MatrixXd> &bounded_axis_estimates,
    std::vector<double> &radius_estimates)
    : model_upsampled_octree_(upsampled_octree),
      scan_kdtree_(new pcl::search::KdTree<pcl::PointXYZ>()),
      mesh_model_(mesh_model),
      model_upsampled_points_(model_upsampled_points),
      center_estimates_(center_estimates),
      direction_estimates_(direction_estimates),
      parameter_estimates_(parameter_estimates),

      bounded_axis_estimates_(bounded_axis_estimates),
      radius_estimates_(radius_estimates) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr meshing_points(
      new pcl::PointCloud<pcl::PointXYZ>());
  for (int i = 0; i < meshing_clouds.size(); i++) {
    (*meshing_points) += *(meshing_clouds.at(i));
  }
  scan_kdtree_->setInputCloud(meshing_points);
}

int ProposalSelection::findMaxIndexInEigenVector(
    const Eigen::VectorXd &vector) {
  double min_value = -1000;
  int max_idx = 0;
  for (int k = 0; k < vector.size(); k++) {
    double cur_value = vector[k];
    if (cur_value > min_value) {
      min_value = cur_value;
      max_idx = k;
    }
  }
  return max_idx;
}

void ProposalSelection::processModelPlanes() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr vertices_model(
      new pcl::PointCloud<pcl::PointXYZ>());
  std::vector<::pcl::Vertices> faces_model;
  pcl::fromPCLPointCloud2(mesh_model_.cloud, *vertices_model);
  faces_model = mesh_model_.polygons;
  for (int i = 0; i < faces_model.size(); i++) {
    std::vector<uint32_t> vertices = faces_model.at(i).vertices;
    pcl::PointXYZ p1 = (*vertices_model)[vertices.at(0)];
    pcl::PointXYZ p2 = (*vertices_model)[vertices.at(1)];
    pcl::PointXYZ p3 = (*vertices_model)[vertices.at(2)];
    Eigen::Vector3d v1(p1.x, p1.y, p1.z);
    Eigen::Vector3d v2(p2.x, p2.y, p2.z);
    Eigen::Vector3d v3(p3.x, p3.y, p3.z);
    Eigen::Vector3d n = (v2 - v1).cross(v3 - v1);

    mesh_area_.push_back(n.norm() / 2.0);
    n.normalize();

    mesh_plane_normals_.push_back(n);
    double d = -(n.x() * p1.x + n.y() * p1.y + n.z() * p1.z);
    mesh_plane_d_.push_back(d);
  }
}

double ProposalSelection::findParameterFromLikelihood(
    const Eigen::VectorXd &params, const Eigen::VectorXd &probabilities,
    int k) {
  // Source:
  // https://stackoverflow.com/questions/1577475/c-sorting-and-keeping-track-of-indexes
  std::vector<double> sorted_probs(
      probabilities.data(),
      probabilities.data() + probabilities.rows() * probabilities.cols());
  std::vector<int> idx(params.size());
  std::iota(idx.begin(), idx.end(), 0);

  std::stable_sort(idx.begin(), idx.end(), [&params](int i1, int i2) {
    return params[i1] > params[i2];
  });

  int idx_to_return;
  if (k < params.size()) {
    idx_to_return = idx[k];
  } else {
    ROS_INFO("Invalid k paramter in findParameterFromLikelihood");
    idx_to_return = params.size() - 1;
  }
  return params[idx_to_return];
}

Eigen::VectorXd ProposalSelection::computePosterior(
    const Eigen::VectorXd &prior, const Eigen::VectorXd &posterior) {
  int size = prior.size();
  Eigen::VectorXd new_probability(size);

  double normalizer = 0;
  for (int i = 0; i < size; i++) {
    normalizer += prior[i] * posterior[i];
  }
  if (normalizer > 10e-5) {
    for (int i = 0; i < size; i++) {
      new_probability[i] = prior[i] * posterior[i] / normalizer;
    }
    return new_probability;
  } else {
    ROS_INFO("Failed to update Posterior - Fall back to prior");
    return prior;
  }
}

void ProposalSelection::selectProposals() {
  // Compute all plane from the building model
  // this->removeInsufficientElements();
  this->processModelPlanes();
  ROS_INFO("Preprocessed Planes");

  std::vector<std::vector<Eigen::VectorXd>> parameter_probabilities(
      parameter_estimates_);

  // Normalize parameters as prior probability
  for (int i = 0; i < parameter_probabilities.size(); i++) {
    for (int j = 0; j < parameter_probabilities.at(i).size(); j++) {
      Eigen::VectorXd para_temp = parameter_probabilities.at(i).at(j);
      para_temp = para_temp.cwiseAbs();
      para_temp.normalize();
      parameter_probabilities.at(i).at(j) = para_temp;
    }
  }
  ROS_INFO("Computed Probabilities");

  for (int i = 0; i < parameter_estimates_.size(); i++) {
    Eigen::Vector3d cur_center = center_estimates_.at(i);
    for (int j = 0; j < parameter_estimates_.at(i).size(); j++) {
      Eigen::VectorXd cur_parameters = parameter_estimates_.at(i).at(j);

      Eigen::Vector3d cur_dir;
      if (j == 0 || j == 1) {
        cur_dir = direction_estimates_.at(i).col(0);
      } else if (j == 2 || j == 3) {
        cur_dir = direction_estimates_.at(i).col(1);
      } else {
        cur_dir = direction_estimates_.at(i).col(2);
      }

      // Posteriors
      std::vector<double> number_of_shared_planes;

      std::vector<int> nn_indices{1};
      std::vector<float> nn_dists{1};
      std::vector<double> scan_alignment_score;
      std::vector<double> model_alignment_score;
      for (int k = 0; k < cur_parameters.size(); k++) {
        double parameter = cur_parameters[k];
        Eigen::Vector3d point = cur_center + parameter * cur_dir;
        double d = -(point.x() * cur_dir.x() + point.y() * cur_dir.y() +
                     point.z() * cur_dir.z());

        // Score for shared planes with the building modle
        double shared_plane_score = 1.0;
        for (int l = 0; l < mesh_plane_d_.size(); l++) {
          Eigen::Vector3d candidate_normal = mesh_plane_normals_.at(l);
          double candidate_d = mesh_plane_d_.at(l);

          if (candidate_normal.dot(cur_dir) < 0) {
            candidate_d *= -1.0;
          }
          double d_diff = std::fabs(d - candidate_d);
          if (std::fabs(candidate_normal.dot(cur_dir)) > 0.99 &&
              d_diff < 0.05) {
            shared_plane_score +=
                ((1.0 - 19.9 * d_diff) * mesh_area_.at(l)) * 0.1;
          }
        }
        number_of_shared_planes.push_back(shared_plane_score);

        pcl::PointXYZ point_pcl(point.x(), point.y(), point.z());
        // Score for scan alignment
        scan_kdtree_->nearestKSearch(point_pcl, 1, nn_indices, nn_dists);
        if (std::sqrt(nn_dists[0]) < 0.05) {
          double scan_alignment = 1.0 - 10.0 * std::sqrt(nn_dists[0]);
          scan_alignment_score.push_back(scan_alignment);
        } else {
          scan_alignment_score.push_back(0.5);
        }

        // Score for model alignment
        model_upsampled_octree_->nearestKSearch(point_pcl, 1, nn_indices,
                                                nn_dists);
        if (std::sqrt(nn_dists[0]) < 0.05) {
          double model_alignment = 1.0 - 10 * std::sqrt(nn_dists[0]);
          model_alignment_score.push_back(model_alignment);
        } else {
          model_alignment_score.push_back(0.5);
        }
      }
      Eigen::VectorXd shared_planes_vec =
          Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
              number_of_shared_planes.data(), number_of_shared_planes.size());
      shared_planes_vec.normalize();

      Eigen::VectorXd scan_alignment_vec =
          Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
              scan_alignment_score.data(), scan_alignment_score.size());
      scan_alignment_vec.normalize();

      Eigen::VectorXd model_alignment_vec =
          Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
              model_alignment_score.data(), model_alignment_score.size());
      model_alignment_vec.normalize();

      Eigen::VectorXd updated_prob1 = this->computePosterior(
          parameter_probabilities.at(i).at(j), shared_planes_vec);
      parameter_probabilities.at(i).at(j) = updated_prob1;

      Eigen::VectorXd updated_prob2 = this->computePosterior(
          parameter_probabilities.at(i).at(j), scan_alignment_vec);
      parameter_probabilities.at(i).at(j) = updated_prob2;

      Eigen::VectorXd updated_prob3 = this->computePosterior(
          parameter_probabilities.at(i).at(j), model_alignment_vec);
      parameter_probabilities.at(i).at(j) = updated_prob3;
    }
  }

  std::vector<int> nn_indices{1};
  std::vector<float> nn_dists{1};


  //Resolve Conflicts with buiding model
  for (int i = 0; i < parameter_probabilities.size(); i++){
    int counter = 0;
    ROS_INFO("Model Conflicts for Element %d", i);
    while(counter < 100){
      ROS_INFO("Start Iteration");
      std::vector<double> most_likely_params_per_elements;
      for (int j = 0; j < parameter_probabilities.at(i).size(); j++) {
        Eigen::VectorXd probability_vec = parameter_probabilities.at(i).at(j);
        int max_idx = this->findMaxIndexInEigenVector(probability_vec);
        double most_likely_para = parameter_estimates_.at(i).at(j)[max_idx];
        most_likely_params_per_elements.push_back(most_likely_para);
      }

      double a1 = most_likely_params_per_elements.at(0);
      double a2 = most_likely_params_per_elements.at(1);
      double b1 = most_likely_params_per_elements.at(2);
      double b2 = most_likely_params_per_elements.at(3);
      double c1 = most_likely_params_per_elements.at(4);
      double c2 = most_likely_params_per_elements.at(5);
      Eigen::Vector3d center = center_estimates_.at(i);
      Eigen::Matrix3d dirs = direction_estimates_.at(i);

      pcl::PointCloud<pcl::PointXYZ>::Ptr upsampled_cloud(
          new pcl::PointCloud<pcl::PointXYZ>());
      this->upsampledStructuredPointCloud(a1, a2, b1, b2, c1, c2, center, dirs,
                                          upsampled_cloud, 0.06);

      std::string save00 =
          "/home/philipp/Schreibtisch/ros_dir/confliction_model_" +
              std::to_string(i) + "_" + std::to_string(counter) + ".ply";
      pcl::io::savePLYFile(save00, *upsampled_cloud);

      Eigen::Vector3d overlap_dir = Eigen::Vector3d::Zero();
      int matches = 0;
      for (const auto &p : (*upsampled_cloud)) {
        //model_upsampled_octree_->nearestKSearch(p, 1, nn_indices, nn_dists);
        //int nearest_idx;
        //float sqrt_dist;
        //model_upsampled_octree_->approxNearestSearch(p, nearest_idx, sqrt_dist);
        model_upsampled_octree_->nearestKSearch(p, 1, nn_indices, nn_dists);
        if (std::sqrt(nn_dists[0]) < 0.03) {
          matches++;
          //pcl::PointXYZ conf_point = (*model_upsampled_points_)[nn_indices[0]];
          //Eigen::Vector3d conf_pont_e(conf_point.x, conf_point.y,
          //                            conf_point.z);
          Eigen::Vector3d conf_pont_e(p.x, p.y,
                                      p.z);
          Eigen::Vector3d center1_e = center_estimates_.at(i);
          Eigen::Vector3d diff_1 = conf_pont_e - center1_e;
          overlap_dir += diff_1;
        }
      }
      double cur_overlap = ((double)matches) / ((double)upsampled_cloud->size());
      overlap_dir.normalize();

      if (cur_overlap <  10e-10){
        break;
      } else {
        int best_k_i = 0;
        double best_kij_score = 0;
        Eigen::Matrix3d dir_i = direction_estimates_.at(i);
        for (int k_i = 0; k_i < 6; k_i++) {
          int col_1 = k_i % 2 == 0 ? k_i / 2 : (k_i - 1) / 2;
          Eigen::Vector3d dir_1 = dir_i.col(col_1);

          Eigen::Vector3d dir_1_oriented;
          if (k_i % 2 != 0) {
            dir_1_oriented = -dir_1;
          }

          double cur_score = overlap_dir.dot(dir_1_oriented);
          if (cur_score > best_kij_score) {
            best_kij_score = cur_score;
            best_k_i = k_i;
          }
        }

        Eigen::VectorXd old_probabilites =
            parameter_probabilities.at(i).at(best_k_i);
        int old_idx = this->findMaxIndexInEigenVector(old_probabilites);
        double old_para = parameter_estimates_.at(i).at(best_k_i)[old_idx];

        Eigen::VectorXd posterior_update(old_probabilites.size());
        for (int v = 0; v < old_probabilites.size(); v++) {
          if (std::fabs(parameter_estimates_.at(i).at(best_k_i)[v]) >=
              (std::fabs(old_para) - 0.015)) {
            posterior_update[v] = 0.0;
          } else {
            posterior_update[v] = 1.0;
          }
        }
        posterior_update.normalize();

        Eigen::VectorXd posterior_updated = this->computePosterior(
            parameter_probabilities.at(i).at(best_k_i),
            posterior_update);

        if ((posterior_updated - parameter_probabilities.at(i).at(best_k_i)).lpNorm<1>() < 10e-10) {
          break;
        }
        parameter_probabilities.at(i).at(best_k_i) = posterior_updated;
        counter++;
      }
    }
  }


  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> conf_point_clouds;
  std::vector<pcl::search::KdTree<pcl::PointXYZ>::Ptr> conf_kd_trees;

  std::vector<std::pair<int, int>> conf_ids;
  std::vector<Eigen::VectorXd> conf_updated_probs;

  std::vector<int> elements_without_conficts;


  ROS_INFO("Element Conficts");
  int counter = 0;
  //Resolve Conflicts between elements
  while (counter < 100) {
    ROS_INFO("Conficting loop start");
    conf_point_clouds.clear();
    conf_kd_trees.clear();

    conf_ids.clear();
    conf_updated_probs.clear();

    for (int i = 0; i < parameter_probabilities.size(); i++) {
      if (std::find(elements_without_conficts.begin(), elements_without_conficts.end(), i) != elements_without_conficts.end()) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr placeholder(
            new pcl::PointCloud<pcl::PointXYZ>());
        placeholder->push_back(pcl::PointXYZ(0, 0, 0));
        pcl::search::KdTree<pcl::PointXYZ>::Ptr placeholder_kd_tree(
            new pcl::search::KdTree<pcl::PointXYZ>());
        placeholder_kd_tree->setInputCloud(placeholder);
        conf_point_clouds.push_back(placeholder);
        conf_kd_trees.push_back(placeholder_kd_tree);
      } else {
        std::vector<double> most_likely_params_per_elements;
        for (int j = 0; j < parameter_probabilities.at(i).size(); j++) {
          Eigen::VectorXd probability_vec = parameter_probabilities.at(i).at(j);
          int max_idx = this->findMaxIndexInEigenVector(probability_vec);
          double most_likely_para = parameter_estimates_.at(i).at(j)[max_idx];
          most_likely_params_per_elements.push_back(most_likely_para);
        }

        double a1 = most_likely_params_per_elements.at(0);
        double a2 = most_likely_params_per_elements.at(1);
        double b1 = most_likely_params_per_elements.at(2);
        double b2 = most_likely_params_per_elements.at(3);
        double c1 = most_likely_params_per_elements.at(4);
        double c2 = most_likely_params_per_elements.at(5);
        Eigen::Vector3d center = center_estimates_.at(i);
        Eigen::Matrix3d dirs = direction_estimates_.at(i);

        pcl::PointCloud<pcl::PointXYZ>::Ptr upsampled_cloud(
            new pcl::PointCloud<pcl::PointXYZ>());
        this->upsampledStructuredPointCloud(a1, a2, b1, b2, c1, c2, center, dirs,
                                            upsampled_cloud, 0.03);
        conf_point_clouds.push_back(upsampled_cloud);

        pcl::search::KdTree<pcl::PointXYZ>::Ptr upsampled_kd_tree(
            new pcl::search::KdTree<pcl::PointXYZ>());
        upsampled_kd_tree->setInputCloud(upsampled_cloud);
        conf_kd_trees.push_back(upsampled_kd_tree);
      }
    }

    ROS_INFO("Save result");
    pcl::PointCloud<pcl::PointXYZ>::Ptr debug_cloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    for (int x = 0; x < conf_point_clouds.size(); x++) {
      (*debug_cloud) += *(conf_point_clouds.at(x));
    }
    std::string save00 =
        "/home/philipp/Schreibtisch/ros_dir/confliction_iter_" +
        std::to_string(counter) + ".ply";
    pcl::io::savePLYFile(save00, *debug_cloud);

    ROS_INFO("Iterate over levels");
    // Iterate over point cloud and fix conflicting dimensions
    for (int l1 = 0; l1 < conf_point_clouds.size(); l1++) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_i = conf_point_clouds.at(l1);
      std::vector<double> overlap_vec;
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> overlap_dirs;
      for (int l2 = 0; l2 < conf_point_clouds.size(); l2++) {
        if (std::find(elements_without_conficts.begin(), elements_without_conficts.end(), l1) != elements_without_conficts.end() ||
            std::find(elements_without_conficts.begin(), elements_without_conficts.end(), l2) != elements_without_conficts.end() ||
            l2 == l1){
          overlap_vec.push_back(0.0);
          overlap_dirs.push_back(
              std::make_pair(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));
          continue;
        }

        pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree_j =
            conf_kd_trees.at(l2);
        int matches = 0;
        Eigen::Vector3d overlap_dir_1 = Eigen::Vector3d::Zero();
        Eigen::Vector3d overlap_dir_2 = Eigen::Vector3d::Zero();
        for (const auto &p : (*cloud_i)) {
          kd_tree_j->nearestKSearch(p, 1, nn_indices, nn_dists);
          if (std::sqrt(nn_dists[0]) < 0.03) {
            matches++;
            pcl::PointXYZ conf_point =
                (*(conf_point_clouds.at(l2)))[nn_indices[0]];
            Eigen::Vector3d conf_pont_e(conf_point.x, conf_point.y,
                                        conf_point.z);
            Eigen::Vector3d center1_e = center_estimates_.at(l1);
            Eigen::Vector3d center2_e = center_estimates_.at(l2);
            Eigen::Vector3d diff_1 = conf_pont_e - center1_e;
            Eigen::Vector3d diff_2 = conf_pont_e - center2_e;
            overlap_dir_1 += diff_1;
            overlap_dir_2 += diff_2;
          }
        }
        double cur_overlap = ((double)matches) / ((double)cloud_i->size());
        overlap_dir_1.normalize();
        overlap_dir_2.normalize();

        overlap_vec.push_back(cur_overlap);
        overlap_dirs.push_back(std::make_pair(overlap_dir_1, overlap_dir_2));
      }

      int max_idx = std::max_element(overlap_vec.begin(), overlap_vec.end()) -
                    overlap_vec.begin();
      double max_overlap = overlap_vec.at(max_idx);
      std::pair<Eigen::Vector3d, Eigen::Vector3d> max_overlap_dir =
          overlap_dirs.at(max_idx);
      ROS_INFO("Check overlap");
      if (max_overlap <  10e-10){
        if (std::find(elements_without_conficts.begin(), elements_without_conficts.end(), l1) == elements_without_conficts.end()){
          elements_without_conficts.push_back(l1);
        }
      }
      else {
        // Get conflicting dimension
        Eigen::Vector3d center_i = center_estimates_.at(l1);
        Eigen::Vector3d center_j = center_estimates_.at(max_idx);
        Eigen::Matrix3d dir_i = direction_estimates_.at(l1);
        Eigen::Matrix3d dir_j = direction_estimates_.at(max_idx);

        std::vector<Eigen::VectorXd> parameters_i = parameter_estimates_.at(l1);
        std::vector<Eigen::VectorXd> parameters_j =
            parameter_estimates_.at(max_idx);

        std::vector<Eigen::VectorXd> probabilites_i =
            parameter_probabilities.at(l1);
        std::vector<Eigen::VectorXd> probabilites_j =
            parameter_probabilities.at(max_idx);

        // Reduce dimension with lower probability
        ROS_INFO("Finde Dimension");
        int best_k_i = 0;
        int best_k_j = 0;
        double best_kij_score = 0;
        Eigen::Vector3d best_dir_1;
        Eigen::Vector3d best_dir_2;
        for (int k_i = 0; k_i < 6; k_i++) {
          for (int k_j = 0; k_j < 6; k_j++) {
            int col_1 = k_i % 2 == 0 ? k_i / 2 : (k_i - 1) / 2;
            int col_2 = k_j % 2 == 0 ? k_j / 2 : (k_j - 1) / 2;

            Eigen::Vector3d dir_1 = dir_i.col(col_1);
            Eigen::Vector3d dir_2 = dir_j.col(col_2);

            Eigen::Vector3d dir_1_oriented;
            Eigen::Vector3d dir_2_oriented;
            if (k_i % 2 != 0) {
              dir_1_oriented = -dir_1;
            }
            if (k_j % 2 != 0) {
              dir_2_oriented = -dir_2;
            }

            double cur_score_1 = max_overlap_dir.first.dot(dir_1_oriented);
            double cur_score_2 = max_overlap_dir.second.dot(dir_2_oriented);
            double cur_score = (cur_score_1 + cur_score_2) / 2.0;
            if (cur_score > best_kij_score) {
              best_kij_score = cur_score;
              best_k_i = k_i;
              best_k_j = k_j;
              best_dir_1 = dir_1;
              best_dir_2 = dir_2;
            }
          }
        }

        double likelihood_1 =
            parameter_probabilities.at(l1).at(best_k_i).maxCoeff();
        double likelihood_2 =
            parameter_probabilities.at(max_idx).at(best_k_j).maxCoeff();

        if (likelihood_2 > likelihood_1) {
          conf_ids.push_back(std::make_pair(l1, best_k_i));
        } else {
          conf_ids.push_back(std::make_pair(max_idx, best_k_j));
        }
      }
    }
    ROS_INFO("Update Posterior");
    // Update corresponding probability vector

    if (conf_ids.size() == 0) {
      break;
    }

    for (int u = 0; u < conf_ids.size(); u++) {
      std::pair<int, int> update_idx = conf_ids.at(u);
      int element_number = update_idx.first;
      int element_dimension = update_idx.second;

      Eigen::VectorXd old_probabilites =
          parameter_probabilities.at(element_number).at(element_dimension);
      int old_idx = this->findMaxIndexInEigenVector(old_probabilites);
      double old_para = parameter_estimates_.at(element_number)
                            .at(element_dimension)[old_idx];

      Eigen::VectorXd posterior_update(old_probabilites.size());
      for (int v = 0; v < old_probabilites.size(); v++) {
        if (std::fabs(parameter_estimates_.at(element_number)
                          .at(element_dimension)[v]) >=
            (std::fabs(old_para) - 0.015)) {
          posterior_update[v] = 0.0;
        } else {
          posterior_update[v] = 1.0;
        }
      }
      posterior_update.normalize();

      Eigen::VectorXd posterior_updated = this->computePosterior(
          parameter_probabilities.at(element_number).at(element_dimension),
          posterior_update);
      parameter_probabilities.at(element_number).at(element_dimension) =
          posterior_updated;
    }

    /*
    for (int d1 = 0; d1 < parameter_probabilities.size(); d1++) {
      ROS_INFO("Element %d", d1);
      for (int d2 = 0; d2 < parameter_probabilities.at(d1).size(); d2++) {
        ROS_INFO("Dim %d", d2);
        Eigen::VectorXd probability_vec = parameter_probabilities.at(d1).at(d2);
        int max_idx = this->findMaxIndexInEigenVector(probability_vec);
        for (int k0 = 0; k0 < probability_vec.size(); k0++) {
          double cur_prob = probability_vec[k0];
          ROS_INFO("Prob %f", cur_prob);
        }
      }
    }
    */

    counter++;
  }

  for (int i = 0; i < parameter_probabilities.size(); i++) {
    for (int j = 0; j < parameter_probabilities.at(i).size(); j++) {
      Eigen::VectorXd probability_vec = parameter_probabilities.at(i).at(j);
      int max_idx = this->findMaxIndexInEigenVector(probability_vec);
      double most_likely_para = parameter_estimates_.at(i).at(j)[max_idx];
      parameter_estimates_.at(i).at(j).resize(1, 1);
      parameter_estimates_.at(i).at(j)[0] = most_likely_para;
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

    int counter_non_zero_elements = 0;
    if (a1.size() > 1) {
      counter_non_zero_elements++;
    }
    if (a2.size() > 1) {
      counter_non_zero_elements++;
    }
    if (b1.size() > 1) {
      counter_non_zero_elements++;
    }
    if (b2.size() > 1) {
      counter_non_zero_elements++;
    }
    if (c1.size() > 1) {
      counter_non_zero_elements++;
    }
    if (c2.size() > 1) {
      counter_non_zero_elements++;
    }

    // Allow for one missed parameter
    if (counter_non_zero_elements <= 4) {
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

void ProposalSelection::upsampledStructuredPointCloud(
    double a1, double a2, double b1, double b2, double c1, double c2,
    Eigen::Vector3d center, Eigen::Matrix3d directions,
    pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud, double tol, double step) {
  a1 -= tol;
  a2 += tol;
  b1 -= tol;
  b2 += tol;
  c1 -= tol;
  c2 += tol;

  int steps_a = (int)((a1 - a2) / step + 1);
  int steps_b = (int)((b1 - b2) / step + 1);
  int steps_c = (int)((c1 - c2) / step + 1);

  for (int i1 = 0; i1 < steps_a; i1++) {
    double cur_a = (a2 + step * i1);
    for (int i2 = 0; i2 < steps_b; i2++) {
      double cur_b = (b2 + step * i2);
      for (int i3 = 0; i3 < steps_c; i3++) {
        double cur_c = (c2 + step * i3);

        if (i1 == 0 || i1 == (steps_a - 1) ||
            i2 == 0 || i2 == (steps_b - 1) ||
            i3 == 0 || i3 == (steps_c - 1)){
          Eigen::Vector3d cur_e = center + directions.col(0) * cur_a +
              directions.col(1) * cur_b +
              directions.col(2) * cur_c;

          pcl::PointXYZ p((float)cur_e.x(), (float)cur_e.y(), (float)cur_e.z());
          result_cloud->push_back(p);
        }
      }
    }
  }
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
