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

void Deviations::init(cgal::Polyhedron &P, const tf::StampedTransform& transform) {
  path_ = params.path;
  // create MeshModel of reference
  cgal::MeshModel::create(P, &reference_mesh);

  // Transform mesh model into localized frame
  Eigen::Matrix3d rotation;
  tf::matrixTFToEigen(transform.getBasis(), rotation);
  Eigen::Vector3d translation;
  tf::vectorTFToEigen(transform.getOrigin(), translation);
  Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
  transformation.block(0, 0, 3, 3) = rotation;
  transformation.block(0, 3, 3, 1) = translation;
  cgal::Transformation ctransformation;
  cgal::eigenTransformationToCgalTransformation(transformation, &ctransformation);
  reference_mesh->transform(ctransformation);
  // process model here, what stays the same between scans

  // Find coplanar facets and create unordered_map Facet ID <-> Plane ID (arbitrary iterated)
  reference_mesh->findAllCoplanarFacets(&facetToPlane, &planeToFacets, 0.01);
  initPlaneMap();
  computeCGALBboxes();
  computeFacetNormals();
}

void Deviations::detectChanges(std::vector<reconstructed_plane> *rec_planes,
                               const PointCloud &reading_cloud,
                               std::vector<reconstructed_plane> *remaining_plane_cloud_vector) {
  // remaining_cloud contains a P.C. with non-segmented points, whereas remaining_plane_cloud_vector
  // a vector of P.C.s of non-associated planes
  /**
   *  Planar Segmentation
   */

  PointMatcherSupport::timer t_segmentation;

  PointCloud remaining_cloud;
  if (params.planarSegmentation == "CGAL") {
    planarSegmentationCGAL(reading_cloud, rec_planes, &remaining_cloud);

  } else if (params.planarSegmentation == "PCL") {
    planarSegmentationPCL(reading_cloud, rec_planes, &remaining_cloud);
  }

  /**
   *  Plane association
   */

  PointMatcherSupport::timer t_association;

  findBestPlaneAssociation(*rec_planes, *reference_mesh, remaining_plane_cloud_vector);

  /**
   *  Find current transformation_map and update transformation_map
   */
  std::unordered_map<int, transformation>
      current_transformation_map;  // the latest transformation result
  PointMatcherSupport::timer t_deviation;
  findPlaneDeviation(&current_transformation_map, 0);
  PointMatcherSupport::timer t_average;
  updateAveragePlaneDeviation(current_transformation_map);
}

/**
 *  This function does basically the same as detectChanges but for the latest map.
 */
void Deviations::detectMapChanges(
    std::vector<reconstructed_plane> *rec_planes, const PointCloud &map_cloud,
    std::vector<reconstructed_plane> *remaining_plane_cloud_vector,
    std::unordered_map<int, transformation> *current_transformation_map) {
  PointCloud remaining_cloud;
  PointMatcherSupport::timer t_segmentation_map;

  if (params.planarSegmentation == "CGAL") {
    planarSegmentationCGAL(map_cloud, rec_planes, &remaining_cloud);
  } else if (params.planarSegmentation == "PCL") {
    planarSegmentationPCL(map_cloud, rec_planes, &remaining_cloud);
  }
  PointMatcherSupport::timer t_association_map;
  findBestPlaneAssociation(*rec_planes, *reference_mesh, remaining_plane_cloud_vector);
  PointMatcherSupport::timer t_deviation_map;
  findPlaneDeviation(current_transformation_map, 1);
}

void Deviations::planarSegmentationPCL(const PointCloud &cloud_in,
                                       std::vector<reconstructed_plane> *rec_planes,
                                       PointCloud *remaining_cloud) const {
  // PCL solution
  // http://pointclouds.org/documentation/tutorials/extract_indices.php#extract-indices
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>(cloud_in)),
      cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>),
      cloud_p(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height << " data points."
            << std::endl;

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(0.01f, 0.01f, 0.01f);
  sor.filter(*cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
            << " data points." << std::endl;

  pcl::ModelCoefficients::Ptr coefficients(
      new pcl::ModelCoefficients());  // estimated plane parameters
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(params.segmentationDistanceThreshold);  // distance to the model

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  int i = 0, nr_points = (int)cloud_filtered->points.size();
  // While 1% of the original cloud is still there
  while (cloud_filtered->points.size() > 0.01 * nr_points) {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() < (uint)params.minNumberOfPlanePoints) {
      std::cerr
          << "Could not estimate a planar model for the remaining dataset. Not enought points."
          << std::endl;
      break;
    }

    std::cerr << "Plane coefficients (ax+by+cz+d = 0): " << coefficients->values[0] << " "
              << coefficients->values[1] << " " << coefficients->values[2] << " "
              << coefficients->values[3] << std::endl;

    // Extract the inliers
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(
        inliers);  // use inlier point indices in filtering with "false" to get only inliers
    extract.setNegative(false);
    extract.filter(*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->size()
              << " data points." << std::endl;

    reconstructed_plane rec_plane;
    rec_plane.coefficients = coefficients->values;  // do we still need this?

    // Compute normalized normal of plane
    Eigen::Vector3d pc_normal(coefficients->values[0], coefficients->values[1],
                              coefficients->values[2]);
    pc_normal.normalize();
    rec_plane.pc_normal = pc_normal;

    // Project PointCloud to the plane, this is optional
    cpt_utils::projectToPlane(cloud_p, coefficients, cloud_p);

    rec_plane.pointcloud = *cloud_p;
    rec_planes->push_back(rec_plane);

    // Create the filtering object
    extract.setNegative(
        true);  // use inlier point indices in filtering with "true" to get cloud except inliers
    extract.filter(*cloud_f);
    cloud_filtered.swap(cloud_f);  // swap cloud_f to cloud_filtered
    i++;
  }
  *remaining_cloud = *cloud_filtered;
}

void Deviations::planarSegmentationCGAL(const PointCloud &cloud,
                                        std::vector<reconstructed_plane> *rec_planes,
                                        PointCloud *remaining_cloud) const {
  // CGAL plane segmentation solution
  // https://doc.cgal.org/latest/Point_set_shape_detection_3/index.html

  // RANSAC faster
  // Region growing better, deterministic

  if (params.planarSegmentationMethod == "RANSAC") {
    std::cout << "Efficient RANSAC" << std::endl;
    runShapeDetection<cgal::Efficient_RANSAC>(cloud, rec_planes, remaining_cloud);
  } else if (params.planarSegmentationMethod == "REGION_GROWING") {
    std::cout << "Region Growing" << std::endl;
    runShapeDetection<cgal::Region_growing>(cloud, rec_planes, remaining_cloud);
  }
}

// This works for RANSAC and Region Growing
template <typename ShapeDetection>
void Deviations::runShapeDetection(const PointCloud &cloud,
                                   std::vector<reconstructed_plane> *rec_planes,
                                   PointCloud *remaining_cloud) const {
  // https://doc.cgal.org/4.13.1/Point_set_shape_detection_3/index.html#title7

  // More about Region Growing
  // https://cgal.geometryfactory.com/CGAL/doc/master/Shape_detection/index.html

  cgal::Pwn_vector points;  // Points with normals.

  // load points from pcl cloud
  for (auto point : cloud.points) {
    cgal::Point_with_normal pwn;
    pwn.first = cgal::ShapeKernel::Point_3(point.x, point.y, point.z);
    points.push_back(pwn);
  }

  // Normal estimation with cgal
  // jet_estimate_normals() is slower than pca_estimate_normals()

  // Estimate normals direction
  const int nb_neighbors = 20;
  CGAL::pca_estimate_normals<Concurrency_tag>(
      points, nb_neighbors,
      CGAL::parameters::point_map(cgal::Point_map()).normal_map(cgal::Normal_map()));

  // Instantiate shape detection engine.
  ShapeDetection shape_detection;
  shape_detection.set_input(points);
  // Registers planar shapes via template method (could also register other shapes)
  shape_detection.template add_shape_factory<cgal::ShapePlane>();
  // Build internal data structures.
  shape_detection.preprocess();

  // Sets parameters for shape detection.
  typename ShapeDetection::Parameters parameters;

  // Sets probability to miss the largest primitive at each iteration.
  // not for region growing
  // parameters.probability = params.segmentationProbability;

  // Detect shapes with at least X points.
  parameters.min_points = params.minNumberOfPlanePoints;
  // Sets maximum Euclidean distance between a point and a shape.
  parameters.epsilon = params.segmentationDistanceThreshold;
  // Sets maximum Euclidean distance between points to be clustered.
  parameters.cluster_epsilon = params.segmentationClusterDistance;
  // Sets maximum normal deviation.
  // 0.9 < dot(surface_normal, point_normal);
  parameters.normal_threshold = params.segmentationNormalThreshold;
  // Perform detection several times and choose result with highest coverage
  typename ShapeDetection::Shape_range shapes = shape_detection.shapes();
  cgal::FT best_coverage = 0;

  // TODO (Hermann) Simplify to only use one iteration
  int number_of_iterations = 1;  // these iteration give almost the same estimates

  for (size_t i = 0; i < number_of_iterations; i++) {
    // Detect registered shapes with parameters.
    shape_detection.detect(parameters);

    // Compute coverage, i.e. ratio of the points assigned to a shape
    cgal::FT coverage = cgal::FT(points.size() - shape_detection.number_of_unassigned_points()) /
                        cgal::FT(points.size());

    // Prints number of assigned shapes and unassigned points
    std::cout << shape_detection.shapes().end() - shape_detection.shapes().begin()
              << " primitives, " << coverage << " coverage" << std::endl;

    // Choose result with highest coverage
    if (coverage > best_coverage) {
      best_coverage = coverage;
      shapes = shape_detection.shapes();
      // return unassigned points
      typename ShapeDetection::Point_index_range unassigned_pts_range =
          shape_detection.indices_of_unassigned_points();
      typename ShapeDetection::Point_index_range::iterator pit = unassigned_pts_range.begin();
      while (pit != unassigned_pts_range.end()) {
        pcl::PointXYZ unassigned_point =
            pcl::PointXYZ(points[*pit].first.x(), points[*pit].first.y(), points[*pit].first.z());
        remaining_cloud->push_back(unassigned_point);
        ++pit;
      }
    }
  }

  // TODO (Hermann) Why does a standard for-loop not work? gives weird compiler errors with a lot
  //                of template problems
  // Characterize shapes
  typename ShapeDetection::Shape_range::iterator shapeIt = shapes.begin();
  while (shapeIt != shapes.end()) {
    cgal::ShapePlane *plane = dynamic_cast<cgal::ShapePlane *>(shapeIt->get());
    cgal::ShapeKernel::Vector_3 normal = plane->plane_normal();
    // std::cout << (*it)->info() << std::endl; // parameters of detected shape
    // std::cout << "Plane with normal " << normal << std::endl;
    const cgal::ShapeKernel::Plane_3 pl3 = static_cast<cgal::ShapeKernel::Plane_3>(*plane);
    // std::cout << "Kernel::Plane_3: " << pl3 << std::endl;

    cgal::FT sum_distances = 0;
    reconstructed_plane rec_plane;
    // Iterates through point indices assigned to each detected shape
    std::vector<std::size_t>::const_iterator index_it =
        (*shapeIt)->indices_of_assigned_points().begin();
    PointCloud pointcloud;
    while (index_it != (*shapeIt)->indices_of_assigned_points().end()) {
      // Retrieve point
      // TODO (Hermann)  DANGEROUS POINTER MAGIC
      const cgal::Point_with_normal &p = *(points.begin() + (*index_it));
      // Adds Euclidean distance between point and shape
      sum_distances += CGAL::sqrt((*shapeIt)->squared_distance(p.first));

      // Generating point cloud
      pcl::PointXYZ pc_point(p.first.x(), p.first.y(), p.first.z());
      pointcloud.push_back(pc_point);
      // Proceeds with next point
      index_it++;
    }
    // Project PointCloud to Plane, this is optional
    cpt_utils::projectToPlane(pointcloud, pl3, &pointcloud);

    rec_plane.pointcloud = pointcloud;
    cgal::FT average_distance = sum_distances / plane->indices_of_assigned_points().size();
    // std::cout << " average distance: " << average_distance << std::endl;
    rec_plane.coefficients.push_back(pl3.a());
    rec_plane.coefficients.push_back(pl3.b());
    rec_plane.coefficients.push_back(pl3.c());
    rec_plane.coefficients.push_back(pl3.d());

    // Compute normalized normal of plane
    Eigen::Vector3d pc_normal(pl3.a(), pl3.b(), pl3.c());
    pc_normal.normalize();
    rec_plane.pc_normal = pc_normal;

    rec_planes->push_back(rec_plane);
    shapeIt++;
  }
}

bool Deviations::associatePlane(cgal::MeshModel &mesh_model, const reconstructed_plane &rec_plane,
                                int *id, double *match_score) {
  bool success = false;
  // TODO check every metrics function used here
  PointCloud cloud = rec_plane.pointcloud;
  // Area ratio check
  double pc_area = cpt_utils::getArea(cloud);

  // Parameters of best fit
  double best_distance_score;
  double best_min_dist;
  double best_avg_dist;
  double best_angle;

  // iterate over all planes
  for (std::pair<int, int> planeAndFacet : planeToFacets) {
    // first check if plane even has size to be associated
    if (plane_map[planeAndFacet.first].area < params.minPolyhedronArea) {
      continue;
    }

    // now check area ratio
    double ratio = pc_area / plane_map[planeAndFacet.first].area;
    if (ratio > params.assocAreaRatioUpperLimit ||
        (plane_map[planeAndFacet.first].area < params.assocAreaLowerLimitThreshold &&
         ratio < params.assocAreaRatioLowerLimit)) {
      continue;
    }

    // now compute all the metrics which we want to use for score, but cancel as soon as a hard
    // threshold is reached put in computationally least expensive order
    double match_score_new = std::numeric_limits<double>::max();
    // std::cout << "Checking Plane ID: " << i->first << std::endl;

    // distance_score of squared distances
    int facet_id = planeAndFacet.second;
    cgal::FT d = 0;
    cgal::Plane plane_of_current_facet = mesh_model.getPlaneFromID(facet_id);

    for (auto point : cloud.points) {
      d += sqrt(
          CGAL::squared_distance(cgal::Point(point.x, point.y, point.z), plane_of_current_facet));
    }
    d = d / cloud.size();
    double distance_score = CGAL::to_double(d);
    if (distance_score > params.matchDistScoreThresh) {
      // std::cout << "distance_score is: " << distance_score << ", but threshold at: " <<
      // params.matchDistScoreThresh << std::endl;
      continue;
    }

    // dist and min_dist (cancels most of assoc, slow)
    double d_min = std::numeric_limits<double>::max();
    double dist = 0;
    // TODO (Hermann) maybe there is a parallellized version?
    for (auto point : cloud.points) {
      double d = std::numeric_limits<double>::max();

      // iterate through all triangles associated to the current plane
      // TODO (Hermann) This could be done more efficiently in an AABB tree
      auto range = planeToFacets.equal_range(planeAndFacet.first);
      for (auto i = range.first; i != range.second; ++i) {
        d = std::min(
            d, CGAL::to_double(CGAL::squared_distance(cgal::Point(point.x, point.y, point.z),
                                                      mesh_model.getTriangleFromID(i->second))));
      }
      d_min = std::min(d, d_min);
      dist += sqrt(d);
    }

    double avg_dist = dist / cloud.size();
    double min_dist = sqrt(d_min);
    if (min_dist > params.matchMinDistThresh) {
      // std::cout << "min_dist is: " << min_dist << ", but threshold at: " <<
      // params.matchMinDistThresh << std::endl;
      continue;
    }
    if (avg_dist > params.matchDistThresh) {
      // std::cout << "avg_dist is: " << avg_dist << ", but threshold at: " <<
      // params.matchDistThresh << std::endl;
      continue;
    }

    // TODO: this needs fix because of different orientation of normals
    // angle (not sure if necessary since already somehow contained in distance_score)
    // we just care about angle of axis angle representation
    double angle = acos(rec_plane.pc_normal.dot(plane_map[planeAndFacet.first].normal));
    // std::cout << "Angle is: " << angle << std::endl;
    // std::cout << "Angles: " << angle << "/ " << std::abs(angle - M_PI) << std::endl;
    // Bring angle to first quadrant
    angle = std::min(angle, std::abs(angle - M_PI));
    // std::cout << "Chosen angle: " << angle << std::endl;
    // std::cout << "P.C. Normal: " << rec_plane.pc_normal << ", Ref. plane normal: " <<
    // plane_map[i->first].normal << std::endl;

    if (angle > params.matchAngleThresh) {
      // std::cout << "angle is: " << angle << " or " << angle_2 << ", but threshold at: " <<
      // params.matchAngleThresh << std::endl;
      continue;
    }

    match_score_new = params.distWeight * avg_dist + params.minDistWeight * min_dist +
                      params.distanceScoreWeight * distance_score + params.angleWeight * angle;
    // std::cout << "Match score new is: " << match_score_new << std::endl;
    // std::cout << "Match score before is: " << *match_score << std::endl;

    // comparing normals -> association rather based on distance since angle is rather
    // a deviation and it's hard to take 2 arguments into account

    // detect if there is no fit (too large deviation) and cancel in that case
    if (match_score_new > params.matchScoreUpperLimit) {
      // std::cout << "match_score is: " << match_score_new << ", but threshold at: " <<
      // params.matchScoreUpperLimit << std::endl;
      continue;
    }

    // lower match_score is better fit!
    if (match_score_new < *match_score) {
      if (match_score_new == plane_map[planeAndFacet.first].match_score) {
        // std::cout << "Match score new is: " << match_score_new << ", map score is: " <<
        // plane_map[i->first].match_score << std::endl; std::cerr << "Pointcloud with same match
        // score. Keeping preceding result." << std::endl;
      } else if (match_score_new < plane_map[planeAndFacet.first].match_score) {
        // std::cout << "Replacing score by new score" << std::endl;
        *id = planeAndFacet.first;
        *match_score = match_score_new;
        success = true;

        // Save parameters for later use
        best_avg_dist = avg_dist;
        best_min_dist = min_dist;
        best_distance_score = distance_score;
        best_angle = angle;
      }
    } else if (match_score_new == *match_score) {
      // std::cout << "Facet with same match score, keeping previous one." << std::endl;
    }
  }
  // Output parameters of best fit
  if (success) {
    std::cout << "PointCloud associated to plane: " << *id << std::endl;
    std::cout << "Match score: " << *match_score << std::endl;
    std::cout << "Min_dist: " << best_min_dist << std::endl;
    std::cout << "Avg_dist: " << best_avg_dist << std::endl;
    std::cout << "Distance score: " << best_distance_score << std::endl;
    std::cout << "Angle: " << best_angle << std::endl;
  }

  return success;
}

void Deviations::findBestPlaneAssociation(
    std::vector<reconstructed_plane> cloud_vector, cgal::MeshModel &mesh_model,
    std::vector<reconstructed_plane> *remaining_plane_cloud_vector) {
  // create queue and apply outlier filter
  std::queue<reconstructed_plane> cloud_queue;
  for (auto cloud : cloud_vector) {
    cpt_utils::removeOutliers(&cloud.pointcloud, 50, 1.0);
    cloud_queue.push(cloud);
  }

  while (!cloud_queue.empty()) {  // while queue not empty
    int id;
    double match_score_new = std::numeric_limits<double>::max();  // smaller is better
    if (associatePlane(mesh_model, cloud_queue.front(), &id, &match_score_new)) {
      if (plane_map[id].associated) {
        cloud_queue.push(plane_map[id].rec_plane);  // re-add pointcloud to queue
      }
      plane_map[id].rec_plane = cloud_queue.front();
      plane_map[id].match_score = match_score_new;
      plane_map[id].associated = true;
      std::cout << "Set plane " << id << std::endl;
    } else {
      // non associated clouds
      std::cout << "Added an unassociated cloud" << std::endl;
      remaining_plane_cloud_vector->push_back(cloud_queue.front());
    }
    cloud_queue.pop();  // remove tested element from queue
  }
}

void Deviations::computeFacetNormals() {
  for (std::pair<int, int> planeAndFacet : planeToFacets) {
    cgal::Vector cnormal = reference_mesh->computeFaceNormal2(
        reference_mesh->getFacetHandleFromId(planeAndFacet.second));
    plane_map[planeAndFacet.first].normal = cgal::cgalVectorToEigenVector(cnormal);
  }
}

/**
 *  PC to Model
 */
void Deviations::findPlaneDeviation(
    std::unordered_map<int, transformation> *current_transformation_map, bool size_check) {
  for (Umiterator umit = plane_map.begin(); umit != plane_map.end(); ++umit) {
    if (umit->second.associated == true) {
      // equal size check in case of full transformations
      if (size_check) {
        double pc_area = cpt_utils::getArea(umit->second.rec_plane.pointcloud);
        if (pc_area < 0.6 * umit->second.area ||
            pc_area > 1.4 * umit->second.area) {  // better use [0.9, 1.1], but keep in mind that
          // current lidar does not have full view
          // std::cout << "Area: " << pc_area << "/ " << umit->second.area << std::endl;
          continue;
        }
      }
      transformation trafo;
      trafo.score = umit->second.match_score;

      // - find translation (also in normal direction is hard since we don't have point association,
      // every point has different translation even in normal direction)
      // - using center point of both planes is only meaningful if we have representative scan of
      // whole plane which is most certainly not the case. So instead of translation better use the
      // distance_score from before

      // Check that pc_normal points in same direction as reference normal
      Eigen::Vector3d pc_normal = umit->second.rec_plane.pc_normal;
      if (pc_normal.dot(umit->second.normal) < 0) {
        pc_normal = -pc_normal;
      }

      /**
       *  Quaternion
       */
      // no need to normalize first
      Eigen::Quaterniond quat = Eigen::Quaterniond::FromTwoVectors(pc_normal, umit->second.normal);
      trafo.quat = quat;

      /**
       *  Angle Axis
       */
      Eigen::AngleAxisd aa(quat);
      trafo.aa = aa;

      // could convert quaternion to euler angles, but these are ambiguous, better work with
      // quaternions

      /**
       *  Translation between the Bbox Center Points
       *  If we have full scan of wall, this might be correct, otherwise not
       */
      cgal::Vector translation = cpt_utils::centerOfBbox(umit->second.bbox) -
                                 cpt_utils::centerOfBbox(umit->second.rec_plane.pointcloud);
      trafo.translation = cgal::cgalVectorToEigenVector(translation);

      // create separate map with only facets ID match_score != 0 (only facets which we associated
      // in current scan) and  transform to each facet
      current_transformation_map->insert(std::make_pair(umit->first, trafo));
    }
  }
}

void Deviations::updateAveragePlaneDeviation(
    const std::unordered_map<int, transformation> &current_transformation_map) {
  // TODO: Do the real update filtering thing here
  // - try to investigate the distribution using the single current_transformation_map and plot them
  // - prediction based on this distribution, but what does it help to know error distribution?
  // - how is update filtering indended to work normally?
  // - now just compute average

  for (auto it = current_transformation_map.begin(); it != current_transformation_map.end(); ++it) {
    // if entry in transformation map does not yet exist for this ID, then just copy
    // current_transformation_map
    if (transformation_map.find(it->first) == transformation_map.end()) {
      transformation_map[it->first] = it->second;
      transformation_map[it->first].count = 1;
      continue;
    }

    transformation trafo;

    trafo.count = transformation_map[it->first].count + 1;

    // update score average
    double current_score = it->second.score;
    trafo.score = transformation_map[it->first].score +
                  (current_score - transformation_map[it->first].score) / trafo.count;
    // update translation average
    Eigen::Vector3d current_translation = it->second.translation;
    trafo.translation =
        transformation_map[it->first].translation +
        (current_translation - transformation_map[it->first].translation) / trafo.count;
    // update rotation average using unit vector rotation
    Eigen::Vector3d v_unit(1, 0, 0);
    v_unit.normalize();
    Eigen::Vector3d u = it->second.quat * v_unit;
    u.normalize();
    Eigen::Vector3d u_avg = transformation_map[it->first].quat * v_unit;
    u_avg.normalize();
    Eigen::Vector3d u_avg_new = u_avg + (u - u_avg) / trafo.count;
    u_avg_new.normalize();

    Eigen::Quaterniond quat = Eigen::Quaterniond::FromTwoVectors(v_unit, u_avg_new);
    trafo.quat = quat;
    Eigen::AngleAxisd aa(quat);
    trafo.aa = aa;

    transformation_map[it->first] = trafo;
  }
}

void Deviations::reset() {
  for (Umiterator umit = plane_map.begin(); umit != plane_map.end(); ++umit) {
    reconstructed_plane rec_plane;
    umit->second.rec_plane = rec_plane;
    umit->second.associated = false;
    umit->second.match_score = std::numeric_limits<double>::max();
  }
}

void Deviations::initPlaneMap() {
  std::cout << "Size of associations is: " << planeToFacets.size() << std::endl;
  for (auto planeAndFacetIt = planeToFacets.begin(); planeAndFacetIt != planeToFacets.end();) {
    polyhedron_plane plane;
    plane.associated = false;
    // getPlane from the first facet of this plane (arbitrary)
    plane.plane = reference_mesh->getPlaneFromID(planeAndFacetIt->second);
    plane_map.insert(std::make_pair(planeAndFacetIt->first, plane));

    // TODO: Decide if using this while-loop or upper_bound as in other functions
    // https://stackoverflow.com/questions/9371236/is-there-an-iterator-across-unique-keys-in-a-stdmultimap
    // Advance to next non-duplicate entry and calculated polyhedron area meanwhile.
    int key = planeAndFacetIt->first;
    double area = 0;
    do {
      area += reference_mesh->getArea(planeAndFacetIt->second);
      ++planeAndFacetIt;
    } while (planeAndFacetIt != planeToFacets.end() &&
             key == planeAndFacetIt->first);  // multidset is ordered
    // save area to struct
    plane_map.at(key).area = area;
  }
}

void Deviations::computeCGALBboxes() {
  for (std::pair<int, int> planeAndFacet : planeToFacets) {
    cgal::Polyhedron::Facet_handle facet_handle =
        reference_mesh->getFacetHandleFromId(planeAndFacet.second);
    CGAL::Bbox_3 bbox =
        CGAL::Polygon_mesh_processing::face_bbox(facet_handle, reference_mesh->getMesh());
    // update bbox
    plane_map.at(planeAndFacet.first).bbox += bbox;
  }
}

}  // namespace deviations
}  // namespace cad_percept
