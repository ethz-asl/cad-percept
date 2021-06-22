#include "cpt_reconstruction/reconstruction_classification.h"

namespace cad_percept {
namespace cpt_reconstruction {
Classification::Classification(ros::NodeHandle nodeHandle1,
                               ros::NodeHandle nodeHandle2)
    : nodeHandle1_(nodeHandle1), nodeHandle2_(nodeHandle2) {
  nodeHandle1.getParam("ClassificationConfigFile1", RF_CONFIG_1_PATH_);
  nodeHandle1.getParam("ClassificationConfigFile2", RF_CONFIG_2_PATH_);
  nodeHandle1.getParam("ClassificationConfigFile3", RF_CONFIG_3_PATH_);
  nodeHandle1.getParam("ClassificationConfigFile4", RF_CONFIG_4_PATH_);
  nodeHandle1.getParam("ClassificationConfigFile5", RF_CONFIG_5_PATH_);

  nodeHandle1.getParam("OutoutClassificationResultFile", RF_RESULT_PATH_);
  nodeHandle1.getParam("CellSize", CELL_SIZE_);
  nodeHandle1.getParam("SmoothingIterations", SMOOTHING_ITERATIONS_);
  nodeHandle1.getParam("MaxFacetLength", MAX_FACET_LENGTH_);
  nodeHandle1.getParam("NumberOfScales", NUMBER_OF_SCALES_);
  nodeHandle1.getParam("NRingQuery", N_RING_QUERY_);

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

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
  pcl::PointCloud<pcl::PointXYZ> all_points;
  for (int i = 0; i < msg.clouds.size(); i++) {
    sensor_msgs::PointCloud2 msg_cloud = msg.clouds.at(i);
    pcl::PCLPointCloud2 pcl_cloud;
    pcl_conversions::toPCL(msg_cloud, pcl_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cur_cloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromPCLPointCloud2(pcl_cloud, *cur_cloud);
    clouds.push_back(cur_cloud);
    all_points += (*cur_cloud);
  }

  // TODO REMOVE
  pcl::io::savePLYFileBinary(
      "/home/philipp/Schreibtisch/ros_dir/scale_space_points.ply", all_points);

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

  Mesh_M mesh;
  pcl::PointCloud<pcl::PointXYZ>::Ptr face_centers(
      new pcl::PointCloud<pcl::PointXYZ>());
  computeReconstructedSurfaceMesh(points, mesh, face_centers);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr centers_kd_tree(
      new pcl::search::KdTree<pcl::PointXYZ>());
  centers_kd_tree->setInputCloud(face_centers);

  //TODO Remove
  std::string filename_mesh =
      "/home/philipp/Schreibtisch/ros_dir/scale_space_mesh_scan.off";
  std::ofstream out_mesh(filename_mesh.c_str());
  CGAL::write_off(out_mesh, mesh);

  std::vector<std::vector<int>> combined_labels;
  int counter_c = 1;
  for (const auto &config_file : all_classifier_paths_){
    std::vector<int> label_indices(mesh.number_of_faces(), -1);
    classifyMesh(counter_c, config_file, mesh, label_indices);
    combined_labels.push_back(label_indices);
    counter_c++;
  }

  int number_of_labels = combined_labels.at(0).size();
  std::vector<int> bagged_labels;

  for (int i = 0; i < number_of_labels; i++){
    std::vector<int> class_votes(6, 0);
    for (const auto &cur_labels : combined_labels) {
      int idx = cur_labels.at(i);
      class_votes.at(idx) += 1;
    }
    int max_idx = std::max_element(class_votes.begin(), class_votes.end()) - class_votes.begin();
    bagged_labels.push_back(max_idx);
  }

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
  double spacing = CGAL::compute_average_spacing<CGAL::Sequential_tag>(
      points, nb_neighbors,
      CGAL::parameters::point_map(
          CGAL::First_of_pair_property_map<PointVectorPair_R>()));

  CGAL::pca_estimate_normals<CGAL::Sequential_tag>(
      points, nb_neighbors,
      CGAL::parameters::point_map(
          CGAL::First_of_pair_property_map<PointVectorPair_R>())
          .normal_map(CGAL::Second_of_pair_property_map<PointVectorPair_R>())
          .neighbor_radius(4. * spacing));

  /*
  std::vector<PointVectorPair_R>::iterator outlier_points_begin =
      CGAL::remove_outliers(
          points, 5,
          CGAL::parameters::point_map(
              CGAL::First_of_pair_property_map<PointVectorPair_R>()));
  points.erase(outlier_points_begin, points.end());
  */

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

  // TODO - CHECK: Mesher Error undefined reference to mpfr_get_emin
  reconstruct.reconstruct_surface(
      CGAL::Scale_space_reconstruction_3::Advancing_front_mesher<Kernel_R>(
          MAX_FACET_LENGTH_));

  assert(reconstruct.number_of_points() == points_only.size());

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
void Classification::classifyMesh(int idx, const std::string config_path, Mesh_M &mesh,
                                  std::vector<int> &label_indices) {
  std::size_t number_of_scales = NUMBER_OF_SCALES_;
  Face_point_map face_point_map(&mesh);
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

  ROS_INFO("Using ETHZ Random Forest Classifier");


  CGAL::Classification::ETHZ_random_forest_classifier classifier(labels,
                                                                 features);
  std::ifstream in_config(config_path.c_str(),
                          std::ios_base::in | std::ios_base::binary);
  classifier.load_configuration(in_config);

  CGAL::Classification::classify_with_graphcut<CGAL::Sequential_tag>(
      mesh.faces(), Face_with_bbox_map(&mesh), labels, classifier,
      generator.neighborhood().n_ring_neighbor_query(N_RING_QUERY_), 0.2f, 1,
      label_indices);

  ROS_INFO("classification done");

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr result(
      new pcl::PointCloud<pcl::PointXYZRGB>());
  int face_idx = 0;
  BOOST_FOREACH (Mesh_M::Face_index f, mesh.faces()) {
    int j = 0;

    int red = 0;
    int green = 0;
    int blue = 0;
    Label_handle label = labels[label_indices[face_idx]];
    if (label == beam) {
      //#magenta -> beam color = np.array([255,255,0])
      red = 255;
      green = 255;
      blue = 0;
    } else if (label == ceiling) {
      //# green -> ceiling color = np.array([0,255,00])
      red = 0;
      green = 255;
      blue = 0;
    } else if (label == clutter) {
      //# black -> clutter color = np.array([50,50,50])
      red = 50;
      green = 50;
      blue = 50;
    } else if (label == column) {
      //# pink -> column color = np.array([255,0,255])
      red = 255;
      green = 0;
      blue = 255;
    } else if (label == floor) {
      //# blue -> floor color = np.array([0, 0, 255])
      red = 0;
      green = 0;
      blue = 255;
    } else if (label == wall) {
      //# red -> wall color = np.array([0, 255, 255])
      red = 255;
      green = 0;
      blue = 0;
    } else {
      ROS_INFO("lable not found");
    }

    CGAL::Vertex_around_face_iterator<Mesh_M> vbegin, vend;
    for (boost::tie(vbegin, vend) =
             vertices_around_face(mesh.halfedge(f), mesh);
         vbegin != vend; ++vbegin) {
      Point_M p1 = mesh.point(*vbegin);
      pcl::PointXYZRGB p_rgb;
      p_rgb.x = (float)p1.x();
      p_rgb.y = (float)p1.y();
      p_rgb.z = (float)p1.z();
      p_rgb.r = (std::uint8_t)red;
      p_rgb.g = (std::uint8_t)green;
      p_rgb.b = (std::uint8_t)blue;
      result->push_back(p_rgb);
    }
    face_idx++;
  }

  // Write result
  std::string output_classified_points = "/home/philipp/Schreibtisch/ros_dir/classification_" + std::to_string(idx) + ".ply";
  pcl::io::savePLYFileBinary(output_classified_points, *result);
  //ROS_INFO("All done");
}

}  // namespace cpt_reconstruction
}  // namespace cad_percept