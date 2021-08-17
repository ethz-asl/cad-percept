#include "cpt_reconstruction/reconstruction_classification.h"

namespace cad_percept {
namespace cpt_reconstruction {
Classification::Classification(ros::NodeHandle nodeHandle1,
                               ros::NodeHandle nodeHandle2)
    : nodeHandle1_(nodeHandle1), nodeHandle2_(nodeHandle2) {
  // Get Parameters
  nodeHandle1.getParam("ClassificationConfigFile1", RF_CONFIG_1_PATH_);
  nodeHandle1.getParam("ClassificationConfigFile2", RF_CONFIG_2_PATH_);
  nodeHandle1.getParam("ClassificationConfigFile3", RF_CONFIG_3_PATH_);
  nodeHandle1.getParam("ClassificationConfigFile4", RF_CONFIG_4_PATH_);
  nodeHandle1.getParam("ClassificationConfigFile5", RF_CONFIG_5_PATH_);
  nodeHandle1.getParam("CellSize", CELL_SIZE_);
  nodeHandle1.getParam("SmoothingIterations", SMOOTHING_ITERATIONS_);
  nodeHandle1.getParam("MaxFacetLength", MAX_FACET_LENGTH_);
  nodeHandle1.getParam("NumberOfScales", NUMBER_OF_SCALES_);
  nodeHandle1.getParam("NRingQuery", N_RING_QUERY_);
  nodeHandle1.getParam("PredictionMethod", PREDICTION_METHOD_);

  // Store paths to RF classifiers
  all_classifier_paths_.push_back(RF_CONFIG_1_PATH_);
  all_classifier_paths_.push_back(RF_CONFIG_2_PATH_);
  all_classifier_paths_.push_back(RF_CONFIG_3_PATH_);
  all_classifier_paths_.push_back(RF_CONFIG_4_PATH_);
  all_classifier_paths_.push_back(RF_CONFIG_5_PATH_);

  subscriber_ = nodeHandle1_.subscribe("clusters", 1000,
                                       &Classification::messageCallback, this);

  publisher_ = nodeHandle2_.advertise<::cpt_reconstruction::classified_shapes>(
      "classified_shapes", 1000);
  ros::spin();
}
void Classification::messageCallback(
    const ::cpt_reconstruction::clusters &msg) {
  ROS_INFO("Received %d", msg.clouds.size());

  // Convert message to PointCloud
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
  for (int i = 0; i < msg.clouds.size(); i++) {
    sensor_msgs::PointCloud2 msg_cloud = msg.clouds.at(i);
    pcl::PCLPointCloud2 pcl_cloud;
    pcl_conversions::toPCL(msg_cloud, pcl_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cur_cloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromPCLPointCloud2(pcl_cloud, *cur_cloud);
    clouds.push_back(cur_cloud);
  }

  // Prepare input for GCAL functionalities
  std::vector<PointVectorPair_R> points;
  for (int i = 0; i < clouds.size(); i++) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cur_cloud = clouds.at(i);
    for (int j = 0; j < cur_cloud->size(); j++) {
      PointVectorPair_R pvp;
      pcl::PointXYZ p = (*cur_cloud)[j];
      pvp.first = Point_R(p.x, p.y, p.z);
      points.push_back(pvp);
    }
  }

  // Compute Reconstructed Mesh
  Mesh_M mesh;
  pcl::PointCloud<pcl::PointXYZ>::Ptr face_centers(
      new pcl::PointCloud<pcl::PointXYZ>());
  computeReconstructedSurfaceMesh(points, mesh, face_centers);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr centers_kd_tree(
      new pcl::search::KdTree<pcl::PointXYZ>());
  centers_kd_tree->setInputCloud(face_centers);

  // Perform predictions with all classifiers
  std::vector<std::vector<int>> combined_labels;
  int counter_c = 1;
  for (const auto &config_file : all_classifier_paths_) {
    std::vector<int> label_indices(mesh.number_of_faces(), -1);
    classifyMesh(counter_c, config_file, mesh, label_indices);
    combined_labels.push_back(label_indices);
    counter_c++;
  }

  int number_of_labels = combined_labels.at(0).size();
  std::vector<int> bagged_labels;

  // Find majority vote
  for (int i = 0; i < number_of_labels; i++) {
    std::vector<int> class_votes(6, 0);
    for (const auto &cur_labels : combined_labels) {
      int idx = cur_labels.at(i);
      class_votes.at(idx) += 1;
    }
    int max_idx = std::max_element(class_votes.begin(), class_votes.end()) -
                  class_votes.begin();
    bagged_labels.push_back(max_idx);
  }

  // Assign cluster to a class
  std::vector<int> nn_indices{1};
  std::vector<float> nn_dists{1};
  std::vector<int> final_votes;
  for (int i = 0; i < clouds.size(); i++) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cur_cloud = clouds.at(i);
    std::vector<double> class_vote(6, 0);

    for (int j = 0; j < cur_cloud->size(); j++) {
      pcl::PointXYZ p = (*cur_cloud)[j];
      centers_kd_tree->nearestKSearch(p, 1, nn_indices, nn_dists);
      if (std::sqrt(nn_dists[0]) < 0.2) {
        int lable = bagged_labels[nn_indices[0]];
        class_vote.at(lable) += 1;
      } else {
        class_vote.at(5) += 1;
      }
    }
    int best_vote = std::max_element(class_vote.begin(), class_vote.end()) -
                    class_vote.begin();
    final_votes.push_back(best_vote);
  }
  ::cpt_reconstruction::classified_shapes class_msg;
  class_msg.robot_positions = msg.robot_positions;
  class_msg.clouds = msg.clouds;
  class_msg.classes = final_votes;
  class_msg.ransac_normal = msg.ransac_normal;
  class_msg.radius = msg.radius;
  class_msg.axis = msg.axis;
  class_msg.id = msg.id;
  publisher_.publish(class_msg);
}

// Source: https://doc.cgal.org/latest/Manual/tuto_reconstruction.html
void Classification::computeReconstructedSurfaceMesh(
    std::vector<PointVectorPair_R> &points, Mesh_M &mesh,
    pcl::PointCloud<pcl::PointXYZ>::Ptr face_centers) {
  const int nb_neighbors = 18;

  // Compute average spacing
  double spacing = CGAL::compute_average_spacing<CGAL::Sequential_tag>(
      points, nb_neighbors,
      CGAL::parameters::point_map(
          CGAL::First_of_pair_property_map<PointVectorPair_R>()));

  // Estimate surface normals
  CGAL::pca_estimate_normals<CGAL::Sequential_tag>(
      points, nb_neighbors,
      CGAL::parameters::point_map(
          CGAL::First_of_pair_property_map<PointVectorPair_R>())
          .normal_map(CGAL::Second_of_pair_property_map<PointVectorPair_R>())
          .neighbor_radius(4. * spacing));

  // Simplify grid
  double cell_size = CELL_SIZE_;
  std::vector<PointVectorPair_R>::iterator unwanted_points_begin =
      CGAL::grid_simplify_point_set(
          points, cell_size,
          CGAL::parameters::point_map(
              CGAL::First_of_pair_property_map<PointVectorPair_R>()));
  points.erase(unwanted_points_begin, points.end());

  std::vector<Point_R> points_only;
  for (int i = 0; i < points.size(); i++) {
    points_only.push_back(points.at(i).first);
  }

  CGAL::Scale_space_surface_reconstruction_3<Kernel_R> reconstruct(
      points_only.begin(), points_only.end());

  // Smooth using 4 iterations of Jet Smoothing
  reconstruct.increase_scale(
      SMOOTHING_ITERATIONS_,
      CGAL::Scale_space_reconstruction_3::Jet_smoother<Kernel_R>());

  // Compute reconstructed mesh
  reconstruct.reconstruct_surface(
      CGAL::Scale_space_reconstruction_3::Advancing_front_mesher<Kernel_R>(
          MAX_FACET_LENGTH_));

  for (int i = 0; i < points_only.size(); i++) {
    Point_R p_r = points_only.at(i);
    mesh.add_vertex(Point_M(p_r.x(), p_r.y(), p_r.z()));
  }

  for (const auto &facet :
       CGAL::make_range(reconstruct.facets_begin(), reconstruct.facets_end())) {
    unsigned long i1 = facet.at(0);
    unsigned long i2 = facet.at(1);
    unsigned long i3 = facet.at(2);

    Eigen::Vector3d p1(points_only.at(i1).x(), points_only.at(i1).y(),
                       points_only.at(i1).z());
    Eigen::Vector3d p2(points_only.at(i2).x(), points_only.at(i2).y(),
                       points_only.at(i2).z());
    Eigen::Vector3d p3(points_only.at(i3).x(), points_only.at(i3).y(),
                       points_only.at(i3).z());
    Eigen::Vector3d center = (p1 + p2 + p3) / 3.0;
    face_centers->push_back(pcl::PointXYZ(center.x(), center.y(), center.z()));

    CGAL::SM_Vertex_index v1(i1);
    CGAL::SM_Vertex_index v2(i2);
    CGAL::SM_Vertex_index v3(i3);
    mesh.add_face(v1, v2, v3);
  }
}

// Source: https://doc.cgal.org/5.0.4/Classification/index.html
void Classification::classifyMesh(int idx, const std::string config_path,
                                  Mesh_M &mesh,
                                  std::vector<int> &label_indices) {
  std::size_t number_of_scales = NUMBER_OF_SCALES_;
  Face_point_map face_point_map(&mesh);

  // Compute point and mesh based features
  Feature_generator generator(mesh, face_point_map, number_of_scales);

  Feature_set features;
  generator.generate_point_based_features(features);
  generator.generate_face_based_features(features);

  Label_set labels;
  Label_handle wall = labels.add("wall");
  Label_handle ceiling = labels.add("ceiling");
  Label_handle beam = labels.add("beam");
  Label_handle column = labels.add("column");
  Label_handle floor = labels.add("floor");
  Label_handle clutter = labels.add("clutter");

  ROS_INFO("Using Random Forest Classifier");

  // Setup RF classifiers
  CGAL::Classification::ETHZ_random_forest_classifier classifier(labels,
                                                                 features);
  // Load config file
  std::ifstream in_config(config_path.c_str(),
                          std::ios_base::in | std::ios_base::binary);
  classifier.load_configuration(in_config);

  // Perform prediction
  if (PREDICTION_METHOD_ == 1) {
    CGAL::Classification::classify_with_graphcut<CGAL::Sequential_tag>(
        mesh.faces(), Face_with_bbox_map(&mesh), labels, classifier,
        generator.neighborhood().n_ring_neighbor_query(N_RING_QUERY_), 0.2f, 1,
        label_indices);
  } else {
    CGAL::Classification::classify_with_local_smoothing<CGAL::Sequential_tag>(
        mesh.faces(), Face_with_bbox_map(&mesh), labels, classifier,
        generator.neighborhood().n_ring_neighbor_query(N_RING_QUERY_),
        label_indices);
  }
  ROS_INFO("Prediction Done");
}

}  // namespace cpt_reconstruction
}  // namespace cad_percept