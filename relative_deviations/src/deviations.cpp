/**
 * Relative Deviations (RD)
 * This is the online, real data solution of Relative Deviations. ICP is executed
 * in separate package cpt_selective_icp.
 * Eventually add a check here if selective ICP was successfully executed!
 */
#include "relative_deviations/deviations.h"

namespace cad_percept {
namespace deviations {

Deviations::Deviations() {}

Deviations::~Deviations() {}

void Deviations::init(const cgal::Polyhedron &P, const std::string &path) {
  path_ = path;
  // create MeshModel of reference
  reference_mesh.init(P);

  // process model here, what stays the same between scans

  // Find coplanar facets and create bimap Facet ID <-> Plane ID (arbitrary iterated)
  reference_mesh.findAllCoplanarFacets(&bimap, 0.01);
  initPlaneMap();
}

void Deviations::detectChanges(std::vector<reconstructed_plane> *rec_planes_publish, const PointCloud &reading_cloud, std::vector<reconstructed_plane> *remaining_cloud_vector, std::unordered_map<int, transformation> *transformation_map) {
  /**
   *  Planar Segmentation
   */

  std::vector<reconstructed_plane> rec_planes;
  std::vector<reconstructed_plane> rec_planes_2;
  planarSegmentationPCL(reading_cloud, &rec_planes);
  planarSegmentationCGAL(reading_cloud, &rec_planes_2);

  // Publish one of them
  *rec_planes_publish = rec_planes; 

  // /* 
  // std::map<int, cgal::Vector> normals;
  // normals = reference_mesh.computeNormals();

  // std::map<int, cgal::Vector>::iterator itr;
  // for (itr = normals.begin(); itr != normals.end(); ++itr) {
  //   std::cout << itr->first << " " << itr->second << std::endl;
  // }
  // */


  // /**
  //  *  Plane association
  //  */

  // findBestPlaneAssociation(rec_planes, reference_mesh, remaining_cloud_vector);

  // /**
  //  *  Find tranformation
  //  */

  // computeFacetNormals(reference_mesh);
  // findPlaneDeviation(transformation_map);

  
}

void Deviations::extractReferenceFacets(const int no_of_points, cgal::Polyhedron &P, std::unordered_set<int> &references, PointCloud *icp_pointcloud) {
  std::cout << "Choosen reference facets:" << std::endl;
  std::unordered_set<int>::iterator uitr;
  for (uitr = references.begin(); uitr != references.end(); uitr++) {
    std::cout << *uitr << std::endl;
  }
  
  // sample reference point cloud from mesh 

  // not sure if it makes sense to use whole colinear plane or just the given facets with a very small filter ratio
  // -> balance between how many points we can match from reading vs. misorientation due to errors in colinear facets
  // find better filter 

  // create unordered set of all neighboring colinear facets
  std::unordered_set<int> references_new;
  // references_new = references; // with trimmed dist outlier filter ratio 0.2

  // with trimmed dist outlier filter ratio 0.3 , this ratio can be as small as possible, but depends on how many associations we have
  // with reference vs. total associations (we only want to keep the correct associations)
  for (auto reference : references) {
    std::cout << reference << std::endl;
    reference_mesh.findCoplanarFacets(reference, &references_new, 0.01);
  }

  std::cout << "Computed reference facets:" << std::endl;
  for (uitr = references_new.begin(); uitr != references_new.end(); uitr++) {
    std::cout << *uitr << std::endl;
  }

  // create sampled point cloud from reference_new
  
  // generated points
  std::vector<cgal::Point> points;
  // create input triangles
  std::vector<cgal::Triangle> triangles;
  for (cgal::Polyhedron::Facet_iterator j = P.facets_begin(); j != P.facets_end(); ++j) {
    if (references_new.find(j->id()) != references_new.end()) {
      triangles.push_back(cgal::Triangle(j->halfedge()->vertex()->point(),
                                   j->halfedge()->next()->vertex()->point(),
                                   j->halfedge()->next()->next()->vertex()->point()));
    }
  }

  // Create the generator, input is the vector of Triangle
  CGAL::Random_points_in_triangles_3<cgal::Point> g(triangles);
  // Get no_of_points random points in cdt
  CGAL::cpp11::copy_n(g, no_of_points, std::back_inserter(points));
  // Check that we have really created no_of_points.
  assert(points.size() == no_of_points);

  for (auto point : points) {
    pcl::PointXYZ cloudpoint;
    cloudpoint.x = (float)point.x();
    cloudpoint.y = (float)point.y();
    cloudpoint.z = (float)point.z();
    icp_pointcloud->push_back(cloudpoint);
  }
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
  writer.write<pcl::PointXYZ> (path_ + "/resources/downsampled.pcd", *cloud_filtered, false);

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

    /*
    std::stringstream ss;
    ss << "/home/julian/megabot_ws/src/cad-percept/relative_deviations/resources/plane_" << i << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str(), *cloud_p, false);
    */

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

void Deviations::associatePlane(cgal::MeshModel &mesh_model, const PointCloud &cloud, int *id, double *match_score) {
  *match_score = 0;
  for (association_bimap::right_const_iterator i = bimap.right.begin(); i != bimap.right.end(); i = bimap.right.upper_bound(i->first)) {
    double match_score_new = 0;
    // calculate a score from squared distances
    std::cout << "Checking Plane ID: " << i->first << std::endl;
    uint facet_id = i->second;
    cgal::FT d = 0;
    for (auto point : cloud.points) {
      d += CGAL::squared_distance(cgal::Point(point.x, point.y, point.z), mesh_model.getPlaneFromID(facet_id));
    }
    d = d/cloud.size();
    std::cout << "Cloud size is: " << cloud.size() << std::endl;
    match_score_new = sqrt(CGAL::to_double(d));
    //std::cout << "Match score new is: " << match_score_new << std::endl;
    //std::cout << "Match score before is: " << *match_score << std::endl;

    // comparing normals -> association rather based on distance since angle is rather
    // a deviation and it's hard to take 2 arguments into account

    // eventually take into account similar sizes
    // find a better match_score, e.g. (1/normalized_size) * distance score

    // detect if there is no fit (too large deviation)

    if (match_score_new < *match_score || *match_score == 0) {
      if (match_score_new == plane_map[i->first].match_score) {
        std::cout << "Match score new is: " << match_score_new << ", map score is: " << plane_map[i->first].match_score << std::endl;
        std::cerr << "Pointcloud with same match score. Keeping preceding result." << std::endl;
      }
      else if (match_score_new < plane_map[i->first].match_score || plane_map[i->first].match_score == 0) {
        //std::cout << "Replacing score by new score" << std::endl;
        *id = i->first;
        *match_score = match_score_new;
      }
    }
    else if (match_score_new == *match_score) {
      std::cout << "Facet with same match score, preceding previous one." << std::endl;
    }
  }
}

void Deviations::findBestPlaneAssociation(const std::vector<reconstructed_plane> &cloud_vector, cgal::MeshModel &mesh_model, std::vector<reconstructed_plane> *remaining_cloud_vector) {
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
    else if (match_score_new < plane_map[id].match_score && match_score_new != 0) {
      cloud_queue.push(plane_map[id].rec_plane); // re-add pointcloud to queue
      plane_map[id].rec_plane = cloud_queue.front();
      plane_map[id].match_score = match_score_new;
    }
    else {
      // non associated clouds
      std::cout << "Added an unassociated cloud" << std::endl;
      remaining_cloud_vector->push_back(cloud_queue.front());
    }
    cloud_queue.pop(); // remove tested element from queue
  }
}

void Deviations::computeFacetNormals(cgal::MeshModel &mesh_model) {
  for (association_bimap::right_const_iterator i = bimap.right.begin(); i != bimap.right.end(); i = bimap.right.upper_bound(i->first)) {
    cgal::Vector cnormal = mesh_model.computeFaceNormal2(mesh_model.getFacetHandle(i->first));
    Eigen::Vector3d normal = cgal::cgalVectorToEigenVector(cnormal);
    plane_map[i->first].normal = normal;
  }
}

void Deviations::findPlaneDeviation(std::unordered_map<int, transformation> *transformation_map) {
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

void Deviations::initPlaneMap() {
  std::cout << "Size of bimap is: " << bimap.size() << std::endl;
  for (association_bimap::right_const_iterator i = bimap.right.begin(); i != bimap.right.end(); ) {
    //std::cout << "First element is: " << i->first << std::endl;

    polyhedron_plane plane;
    plane.plane = reference_mesh.getPlaneFromID(i->second); // getPlane from the first facet of this plane (arbitrary)
    plane_map.insert(std::make_pair(i->first, plane));

    // TODO: Decide if using this while-loop or upper_bound as in other functions
    // https://stackoverflow.com/questions/9371236/is-there-an-iterator-across-unique-keys-in-a-stdmultimap
    // Advance to next non-duplicate entry.
    int key = i->first;
    do {
      ++i;
    } while (i != bimap.right.end() && key == i->first);
  }
}

}
}