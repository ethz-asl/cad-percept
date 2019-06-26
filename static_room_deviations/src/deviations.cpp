#include "static_room_deviations/deviations.h"

namespace cad_percept {
namespace room_deviations {

Deviations::Deviations() {
  // process model here, what stays the same between scans
  cgal::Polyhedron P;
  cgal::Polyhedron P_deviated;
  cgal::build_sample_polyhedrons(&P, &P_deviated);
  cgal::sample_pc_from_mesh(P_deviated, 3000, 0.01, &pointcloud, "P_deviated");

  // create MeshModel of reference
  reference_mesh.init(P);

  // get ref_pc from model mesh for ICP
  cgal::sample_pc_from_mesh(P, pointcloud.points.size(), 0.0, &ref_pc, "P");

  // Merge coplanar facets and create plane_map
  // Since we want to keep initial MeshModel, get Polyhedron and initialize a new one
  cgal::Polyhedron old_P_merged; // can not be published anymore, since it is not a triangle mesh
  std::multimap<int, int> old_merge_associations;
  reference_mesh.mergeCoplanarFacets(&old_P_merged, &old_merge_associations);
  reference_mesh_merged.init(old_P_merged); // attention, this initializes the facet ID's new (same order, but beginning from 0)
  updateAssociations(old_merge_associations, &merge_associations);
  initPlaneMap(reference_mesh_merged);
}

Deviations::~Deviations() {}

void Deviations::detectChanges(std::vector<reconstructed_plane> *rec_planes_publish, PointCloud *reading_cloud, PointCloud *icp_cloud, std::ifstream &ifs_icp_config, std::ifstream &ifs_normal_filter) {
  // transform reading pointcloud a little bit to test ICP
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << 0.5, 0.1, 0.2;
  float theta = M_PI*0.01;
  transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
  transformPointCloud(&pointcloud, transform);
  pcl::io::savePCDFileASCII("/home/julian/cadify_ws/src/mt_utils/static_room_deviations/resources/P_deviated_transformed.pcd", pointcloud);


  /**
   *  ICP
   */ 

  // convert point clouds to DP
  DP dppointcloud = pointCloudToDP(pointcloud);
  DP dpref = pointCloudToDP(ref_pc);
  // DP dppointcloud = PointMatcher_ros::rosMsgToPointMatcherCloud<double>(cloud);
  // DP dpref(DP::load("P.pcd"));
  // DP dppointcloud(DP::load("P_deviated_transformed.pcd"));

  loadICPConfig(ifs_icp_config, ifs_normal_filter);
  // Compute the transformation to express data in ref
  PM::TransformationParameters T = icp_(dppointcloud, dpref);
  // Transform data to express it in ref
  DP dppointcloud_out(dppointcloud);
  icp_.transformations.apply(dppointcloud_out, T);
  dppointcloud_out.save("/home/julian/cadify_ws/src/mt_utils/static_room_deviations/resources/P_icp.pcd");
  std::cout << "Final ICP transformation: " << std::endl << T << std::endl;

  PointCloud pointcloud_out = dpToPointCloud(dppointcloud_out);
  *reading_cloud = pointcloud;
  *icp_cloud = pointcloud_out;
  getResidualError(dpref, dppointcloud_out);

  /**
   *  Planar Segmentation
   */

  std::vector<reconstructed_plane> rec_planes;
  std::vector<reconstructed_plane> rec_planes_2;
  planarSegmentationPCL(pointcloud_out, &rec_planes);
  planarSegmentationCGAL(pointcloud_out, &rec_planes_2);

  // Publish one of them
  *rec_planes_publish = rec_planes; 

  /* 
  std::map<int, cgal::Vector> normals;
  normals = reference_mesh.computeNormals();

  std::map<int, cgal::Vector>::iterator itr;
  for (itr = normals.begin(); itr != normals.end(); ++itr) {
    std::cout << itr->first << " " << itr->second << std::endl;
  }
  */

  // reference_mesh.printFacetsOfHalfedges();
  cgal::Polyhedron P_merged = reference_mesh_merged.getMesh();
  // CGAL::draw(P_merged); // requires Qt5
  std::ofstream off_file("/home/julian/cadify_ws/src/mt_utils/static_room_deviations/resources/merged.off", std::ios::binary);
  if(CGAL::write_off(off_file, P_merged)) {
    std::cout << "Merged Mash written to file" << std::endl;
  }

  /**
   *  Plane association
   */

  findBestPlaneAssociation(rec_planes, reference_mesh_merged);

  /**
   *  Find tranformation
   */

  computeFacetNormals(reference_mesh_merged);
  std::unordered_map<int, transformation> transformation_map;
  findPlaneDeviation(reference_mesh_merged, &transformation_map);
  reset();
}

void Deviations::updateAssociations(std::multimap<int, int> &merge_associations, std::multimap<int, int> *new_merge_associations) const {
  int i = 0;
  for(Mmiterator it = merge_associations.begin(); it != merge_associations.end(); it = merge_associations.upper_bound(it->first)) {
    int key = it->first;
    std::cout << "Key: " << key << std::endl;

    auto iit = merge_associations.equal_range(key);
    new_merge_associations->insert(std::make_pair(i, it->first));
    for (auto itr = iit.first; itr != iit.second; ++itr) {
      new_merge_associations->insert(std::make_pair(i, itr->second));
    }
    ++i; // since merge_associations is an ordered list according to facet order
  }

  // Print map
  for (Mmiterator it = new_merge_associations->begin(); it != new_merge_associations->end(); it = new_merge_associations->upper_bound(it->first)) {
    std::cout << "Merge Association Map: " << it->first << ": ";
    auto iit = new_merge_associations->equal_range(it->first);
    for (auto itr = iit.first; itr != iit.second; ++itr) {
      std::cout << itr->second << ", ";
    }
    std::cout << std::endl;
  }
}

void Deviations::transformPointCloud(PointCloud *pointcloud, const Eigen::Affine3f &transform) const {
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::transformPointCloud (*pointcloud, *pointcloud, transform);
}

DP Deviations::pointCloudToDP(const PointCloud &pointcloud) const {
  // alternatively use matrix3dEigenToPointMatcher (octomap_compare)
  // or rosMsgToPointMatcherCloud (libpointmatcher_ros)
  const int dimFeatures = 4;

  PM::Matrix feat(dimFeatures, pointcloud.points.size());
  for (uint i = 0; i < pointcloud.points.size(); ++i) {
    feat(0, i) = pointcloud[i].x;
    feat(1, i) = pointcloud[i].y;
    feat(2, i) = pointcloud[i].z;
    feat(3, i) = 1.0;
  }

  DP::Labels featLabels;
  featLabels.push_back(DP::Label("x", 1));
  featLabels.push_back(DP::Label("y", 1));
  featLabels.push_back(DP::Label("z", 1));
  featLabels.push_back(DP::Label("pad", 1));

  DP dppointcloud = DP(feat, featLabels); // construct a point cloud from existing features without any descriptor

  return dppointcloud;
}

PointCloud Deviations::dpToPointCloud(const DP &dppointcloud) const {
  const size_t n_points = dppointcloud.getNbPoints();
  PointCloud cloud;
  for (uint i = 0; i < n_points; ++i) {
    pcl::PointXYZ point;
    point.x = dppointcloud.features(0, i);
    point.y = dppointcloud.features(1, i);
    point.z = dppointcloud.features(2, i);
    cloud.push_back(point);
  }
  return cloud;
}

void Deviations::loadICPConfig(std::ifstream &ifs_icp_config, std::ifstream &ifs_normal_filter) {
  if (ifs_icp_config.good()) {
    LOG(INFO) << "Loading ICP configurations";
    icp_.loadFromYaml(ifs_icp_config);
  }
  else {
    LOG(WARNING) << "Could not open ICP configuration. Using default configuration.";  
    icp_.setDefault();
  }

  // Load a data point filter
  if (ifs_normal_filter.good()) {
    LOG(INFO) << "Loading ICP normal filter"; 
    normal_filter_ = PM::DataPointsFilters(ifs_normal_filter);
  } 
  else {
    LOG(WARNING) << "Could not open normal filter";
  }
}

void Deviations::getResidualError(const DP &dpref, const DP &dppointcloud_out) {
  // https://github.com/ethz-asl/libpointmatcher/issues/193

  // reuse the same module used for the icp object
  // in case we need new matching module:
  // https://github.com/ethz-asl/libpointmatcher/blob/master/examples/icp_advance_api.cpp
  // https://github.com/ethz-asl/libpointmatcher/issues/193
  // initiate matching with unfiltered point cloud
  DP ref = dpref;
  DP pointcloud_out = dppointcloud_out;
  normal_filter_.apply(ref);
  normal_filter_.apply(pointcloud_out);
  icp_.matcher->init(ref);
  // extract closest points
  PM::Matches matches = icp_.matcher->findClosests(pointcloud_out);
  // weight paired points
  PM::OutlierWeights outlierWeights = icp_.outlierFilters.compute(pointcloud_out, ref, matches);
  // get error, why is error smaller if outlier filter ratio is smaller and result completely misaligned?!
  float error = icp_.errorMinimizer->getResidualError(pointcloud_out, ref, outlierWeights, matches);
  std::cout << "Final residual error: " << error << std::endl;
}

void Deviations::planarSegmentationPCL(const PointCloud &cloud_in, std::vector<reconstructed_plane> *rec_planes) const {
  // PCL solution
  // http://pointclouds.org/documentation/tutorials/extract_indices.php#extract-indices
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ> (cloud_in)), cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height << " data points." << std::endl;

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(0.01f, 0.01f, 0.01f);
  sor.filter(*cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

  // Write the downsampled version to disk
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("/home/julian/cadify_ws/src/mt_utils/static_room_deviations/resources/table_scene_lms400_downsampled.pcd", *cloud_filtered, false);

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients ()); // estimated plane parameters
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.05);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  int i = 0, nr_points = (int) cloud_filtered->points.size ();
  // While 1% of the original cloud is still there
  while (cloud_filtered->points.size () > 0.01 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    std::cerr << "Plane coefficients (ax+by+cz+d = 0): " << coefficients->values[0] << " " 
                                                         << coefficients->values[1] << " "
                                                         << coefficients->values[2] << " " 
                                                         << coefficients->values[3] << std::endl;

    // Extract the inliers
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers); // use inlier point indices in filtering with "false" to get only inliers
    extract.setNegative(false);
    extract.filter(*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    reconstructed_plane rec_plane;
    rec_plane.coefficients = coefficients->values; //do we still need this?

    // Compute normalized normal of plane
    Eigen::Vector3d pc_normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    pc_normal.normalize();
    rec_plane.pc_normal = pc_normal;

    rec_plane.pointcloud = *cloud_p;
    rec_planes->push_back(rec_plane);

    std::stringstream ss;
    ss << "/home/julian/cadify_ws/src/mt_utils/static_room_deviations/resources/plane_" << i << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str(), *cloud_p, false);

    // Create the filtering object
    extract.setNegative(true); // use inlier point indices in filtering with "true" to get cloud except inliers
    extract.filter(*cloud_f); 
    cloud_filtered.swap(cloud_f); // swap cloud_f to cloud_filtered
    i++;
  }

}

void Deviations::planarSegmentationCGAL(const PointCloud &cloud, std::vector<reconstructed_plane> *rec_planes) const {
  // CGAL plane segmentation solution
  // https://doc.cgal.org/latest/Point_set_shape_detection_3/index.html

  // RANSAC faster
  // Region growing better, deterministic
  
  std::cout << "Efficient RANSAC" << std::endl;
  runShapeDetection<cgal::Efficient_ransac>(cloud, rec_planes);
  
  std::cout << "Region Growing" << std::endl;
  runShapeDetection<cgal::Region_growing>(cloud, rec_planes);
}

// This works for RANSAC and Region Growing
template <typename ShapeDetection>
void Deviations::runShapeDetection(const PointCloud &cloud, std::vector<reconstructed_plane> *rec_planes) const {
  // https://doc.cgal.org/4.13.1/Point_set_shape_detection_3/index.html#title7  

  // Points with normals.
  cgal::Pwn_vector points;

  // load points from pcl cloud
  for (auto point : cloud.points) {
    cgal::Point_with_normal pwn;
    pwn.first = cgal::ShapeKernel::Point_3(point.x, point.y, point.z);
    points.push_back(pwn);
  }

  // Is this needed at all?
  // CGAL::First_of_pair_property_map<cgal::Point_with_normal> point_map;
  // point_map = CGAL::make_first_of_pair_property_map(points[0]);

  // Normal estimation with cgal
  // jet_estimate_normals() is slower than pca_estimate_normals()

  // Estimate normals direction
  const int nb_neighbors = 10;
  CGAL::pca_estimate_normals<Concurrency_tag>(points, nb_neighbors,
                                              CGAL::parameters::point_map(cgal::Point_map()).
                                                                normal_map(cgal::Normal_map()));
  
  /* not necessary
  // Orient normals, successfully oriented points first, then points with unoriented normals (iterator begin)
  cgal::Pwn_vector::iterator unoriented_points_begin = CGAL::mst_orient_normals(points, nb_neighbors,
                                                                                            CGAL::parameters::point_map(cgal::Point_map()).
                                                                                                              normal_map(cgal::Normal_map()));

  // Delete points with unoriented normal  
  points.erase(unoriented_points_begin, points.end());            
  */                                                                                              

  std::cout << "Size of pwn_vector: " << points.size() << std::endl;

  // Instantiate shape detection engine.
  ShapeDetection shape_detection;

  // Provide input data.
  shape_detection.set_input(points);

  // Registers planar shapes via template method (could also register other shapes)
  shape_detection.template add_shape_factory<cgal::ShapePlane>();

  // Measures time before setting up the shape detection.
  CGAL::Timer time;
  time.start();

  // Build internal data structures.
  shape_detection.preprocess();

  // Measures time after preprocessing.
  time.stop();

  std::cout << "preprocessing took: " << time.time() * 1000 << "ms" << std::endl;

  // Sets parameters for shape detection.
  typename ShapeDetection::Parameters parameters;

  // Sets probability to miss the largest primitive at each iteration.
  // not for region growing
  // parameters.probability = 0.05;
   
  // Detect shapes with at least 500 points.
  parameters.min_points = 30;

  // Sets maximum Euclidean distance between a point and a shape.
  parameters.epsilon = 0.05;
 
  // Sets maximum Euclidean distance between points to be clustered.
  parameters.cluster_epsilon = 0.5;
 
  // Sets maximum normal deviation.
  // 0.9 < dot(surface_normal, point_normal); 
  parameters.normal_threshold = 0.9;   

  // Perform detection several times and choose result with highest coverage
  typename ShapeDetection::Shape_range shapes = shape_detection.shapes();
  cgal::FT best_coverage = 0;

  for (size_t i = 0; i<3; i++) {
    // Reset timer
    time.reset();
    time.start();

    // Detect registered shapes with default parameters.
    shape_detection.detect(parameters);

    // Measures time after detection
    time.stop();

    // Compute coverage, i.e. ratio of the points assigned to a shape
    cgal::FT coverage = cgal::FT(points.size() - shape_detection.number_of_unassigned_points()) / cgal::FT(points.size());
  
    // Prints number of assigned shapes and unassigned points
    std::cout << "time: " << time.time() * 1000 << "ms" << std::endl;
    std::cout << shape_detection.shapes().end() - shape_detection.shapes().begin() << " primitives, "
              << coverage << " coverage" << std::endl;
  
    // Choose result with highest coverage
    if (coverage > best_coverage) {
      best_coverage = coverage;
      shapes = shape_detection.shapes();
    }
  }

  // Detect shapes
  typename ShapeDetection::Shape_range::iterator it = shapes.begin();

  while (it != shapes.end()) {
    cgal::ShapePlane* plane = dynamic_cast<cgal::ShapePlane*>(it->get());
    cgal::ShapeKernel::Vector_3 normal = plane->plane_normal();
    std::cout << (*it)->info() << std::endl;
    std::cout << "Plane with normal " << normal << std::endl;
    std::cout << "Kernel::Plane_3: " << static_cast<cgal::ShapeKernel::Plane_3>(*plane) << std::endl;
    const cgal::ShapeKernel::Plane_3 pl3 = static_cast<cgal::ShapeKernel::Plane_3>(*plane);

    // Sums distances of points to detected shapes
    cgal::FT sum_distances = 0;
    reconstructed_plane rec_plane;
    // Iterates through point indices assigned to each detected shape
    std::vector<std::size_t>::const_iterator index_it = (*it)->indices_of_assigned_points().begin();
    while (index_it != (*it)->indices_of_assigned_points().end()) {
      // Retrieve point
      const cgal::Point_with_normal &p = *(points.begin() + (*index_it));
      // Adds Euclidean distance between point and shape
      sum_distances += CGAL::sqrt((*it)->squared_distance(p.first));

      // Generating point cloud
      pcl::PointXYZ pc_point(p.first.x(), p.first.y(), p.first.z());
      rec_plane.pointcloud.push_back(pc_point);
      // Proceeds with next point
      index_it++;
    }
    cgal::FT average_distance = sum_distances / plane->indices_of_assigned_points().size();
    std::cout << " average distance: " << average_distance << std::endl;
    rec_plane.coefficients.push_back(pl3.a());
    rec_plane.coefficients.push_back(pl3.b());
    rec_plane.coefficients.push_back(pl3.c());
    rec_plane.coefficients.push_back(pl3.d());

    // Compute normalized normal of plane
    Eigen::Vector3d pc_normal(pl3.a(), pl3.b(), pl3.c());
    pc_normal.normalize();
    rec_plane.pc_normal = pc_normal;

    rec_planes->push_back(rec_plane);
    it++;
  }
}

void Deviations::associatePlane(const cgal::MeshModel &mesh_model, const PointCloud &cloud, int *id, double *match_score) {
  cgal::Polyhedron P = mesh_model.getMesh();
  *match_score = 0;
  for (cgal::Polyhedron::Facet_iterator j = P.facets_begin(); j != P.facets_end(); ++j) {
    double match_score_new = 0;
    // calculate a score from squared distances
    std::cout << "Checking facet ID: " << j->id() << std::endl;
    cgal::FT d = 0;
    for (auto point : cloud.points) {
      d += CGAL::squared_distance(cgal::Point(point.x, point.y, point.z), mesh_model.getPlane(j));
    }
    d = d/cloud.size();
    std::cout << "Cloud size is: " << cloud.size() << std::endl;
    match_score_new = sqrt(CGAL::to_double(d));
    std::cout << "Match score new is: " << match_score_new << std::endl;
    std::cout << "Match score before is: " << *match_score << std::endl;

    // comparing normals -> association rather based on distance since angle is rather
    // a deviation and it's hard to take 2 arguments into account

    // eventually take into account similar sizes
    // find a better match_score, e.g. (1/normalized_size) * distance score

    // detect if there is no fit (too large deviation)

    if (match_score_new < *match_score || *match_score == 0) {
      if (match_score_new == plane_map[j->id()].match_score) {
        std::cout << "Match score new is: " << match_score_new << ", map score is: " << plane_map[j->id()].match_score << std::endl;
        std::cerr << "Pointcloud with same match score. Keeping preceding result." << std::endl;
      }
      else if (match_score_new < plane_map[j->id()].match_score || plane_map[j->id()].match_score == 0) {
        std::cout << "Replacing score by new score" << std::endl;
        *id = j->id();
        *match_score = match_score_new;
      }
    }
    else if (match_score_new == *match_score) {
      std::cout << "Facet with same match score, preceding previous one." << std::endl;
    }
  }
}

void Deviations::findBestPlaneAssociation(const std::vector<reconstructed_plane> &cloud_vector, const cgal::MeshModel &mesh_model) {
  cgal::Polyhedron P = mesh_model.getMesh();

  // create queue
  std::queue<reconstructed_plane> cloud_queue;
  for (auto cloud : cloud_vector) {
    cloud_queue.push(cloud);
  }

  while (!cloud_queue.empty()) { // while queue not empty
    int id;
    double match_score_new = 0;
    associatePlane(mesh_model, cloud_queue.front().pointcloud, &id, &match_score_new);
    if (plane_map[id].match_score == 0) {
      // first time associated to this plane
      plane_map[id].rec_plane = cloud_queue.front();
      plane_map[id].match_score = match_score_new;
    }
    else if (match_score_new < plane_map[id].match_score) {
      cloud_queue.push(plane_map[id].rec_plane); // re-add pointcloud to queue
      plane_map[id].rec_plane = cloud_queue.front();
      plane_map[id].match_score = match_score_new;
    }
    cloud_queue.pop(); // remove tested element from queue
  }
}

void Deviations::computeFacetNormals(const cgal::MeshModel &mesh_model) {
  cgal::Polyhedron P = mesh_model.getMesh();
  for (cgal::Polyhedron::Facet_iterator j = P.facets_begin(); j != P.facets_end(); ++j) {
    cgal::Vector cnormal = mesh_model.computeFaceNormal2(j);
    Eigen::Vector3d normal = cgal::cgalVectorToEigenVector(cnormal);
    plane_map[j->id()].normal = normal;
  }
}

void Deviations::findPlaneDeviation(const cgal::MeshModel &mesh_model, std::unordered_map<int, transformation> *transformation_map) {
  cgal::Polyhedron P = mesh_model.getMesh();
  for (Umiterator umit = plane_map.begin(); umit != plane_map.end(); ++umit) {
    if (umit->second.match_score != 0) {
      std::cout << "Process plane ID: " << umit->first << std::endl;
      transformation trafo;
      // set distance_score to match_score
      trafo.distance_score = umit->second.match_score; // since we set it equal before

      // - find translation (also in normal direction is hard since we don't have point association,
      // every point has different translation even in normal direction)
      // - using center point of both planes is only meaningful if we have representative scan of whole plane
      // which is most certainly not the case. So instead of translation better use the distance_score from before

      // find angle

      // Angle Axis
      Eigen::Vector3d axis = umit->second.rec_plane.pc_normal.cross(umit->second.normal);
      // std::cout << "Vector pc: " << umit->second.rec_plane.pc_normal << ", Vector facet: " << umit->second.normal << std::endl;
      axis.normalize();
      // std::cout << "Axis: " << axis << std::endl;
      double angle = acos(umit->second.rec_plane.pc_normal.dot(umit->second.normal));
      // std::cout << "Angle: " << angle << std::endl;
      Eigen::AngleAxisd aa(angle, axis);
      trafo.aa = aa;

      // To Quaternion
      Eigen::Quaterniond quat(aa);
      trafo.quat = quat;

      // could convert quaternion to euler angles, but these are ambiguous, better work with
      // quaternions 

      // create separate map with only facets ID match_score != 0 (only facets which we associated in current scan)
      // and  transform to each facet
      transformation_map->insert(std::make_pair(umit->first, trafo));

    }
  }
}

void Deviations::reset() {
  for (Umiterator umit = plane_map.begin(); umit != plane_map.end(); ++umit) {
    reconstructed_plane rec_plane;
    umit->second.rec_plane = rec_plane;
    umit->second.match_score = 0; 
  }
}

void Deviations::initPlaneMap(const cgal::MeshModel &mesh_model) {
  cgal::Polyhedron P = mesh_model.getMesh();
  for (uint j = 0; j < P.size_of_facets(); ++j) {
    polyhedron_plane plane;
    plane_map.insert(std::make_pair(j, plane));
  }
}

}
}