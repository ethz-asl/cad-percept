#include "plane_extraction/plane_extraction.h"

namespace cad_percept {
namespace matching_algorithms {

using namespace pcl;

void PlaneExtractor::rhtPlaneExtraction(std::vector<PointCloud<PointXYZ>> &extracted_planes,
                                        std::vector<Eigen::Vector3d> &plane_normals,
                                        PointCloud<PointXYZ> lidar_scan, std::string tf_map_frame,
                                        ros::Publisher &plane_pub) {
  std::cout << "///////////////////////////////////////////////" << std::endl;
  std::cout << "         RHT Plane Extraction started          " << std::endl;
  std::cout << "///////////////////////////////////////////////" << std::endl;

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  plane_pub = nh.advertise<sensor_msgs::PointCloud2>("extracted_planes", 1, true);

  int accumulator_choice = nh_private.param<int>("AccumulatorChoice", 2);
  double rho_resolution = nh_private.param<double>("AccumulatorRhoResolution", 0.1);
  double theta_resolution = nh_private.param<double>("AccumulatorThetaResolution", 0.04);
  double psi_resolution = nh_private.param<double>("AccumulatorPsiResolution", 0.04);
  int min_vote_threshold = nh_private.param<int>("AccumulatorThreshold", 0);
  int k_of_maxima_suppression = nh_private.param<int>("AccumulatorKMaxSuppress", 14);

  int max_iteration = nh_private.param<int>("RHTMaxIter", 100000);
  double tol_distance_between_points = nh_private.param<double>("RHTTolDist", 3);
  double min_area_spanned = nh_private.param<double>("RHTMinArea", 0.5);
  int num_main_planes = nh_private.param<int>("RHTPlaneNumber", 1);

  PointXYZ origin(0, 0, 0);
  double max_distance_to_point = 0;
  for (auto lidar_scan_point : lidar_scan) {
    if ((double)geometry::distance(origin, lidar_scan_point) > max_distance_to_point)
      max_distance_to_point = (double)geometry::distance(origin, lidar_scan_point);
  }

  // Settings for accumulator discretization (rho, theta, psi)
  HoughAccumulator *accumulator;
  Eigen::Vector3d min_coord(0, 0, -(double)M_PI / 2);
  Eigen::Vector3d max_coord(max_distance_to_point, (double)2 * M_PI, (double)M_PI / 2);
  Eigen::Vector3d bin_size(rho_resolution, theta_resolution, psi_resolution);
  switch (accumulator_choice) {
    case 1:
      accumulator = new ArrayAccumulator(min_coord, bin_size, max_coord);
      break;
    case 2:
      accumulator = new BallAccumulator(min_coord, bin_size, max_coord);
      break;
    default:
      std::cout << "No valid accumulator to number " << accumulator_choice << ", return ..."
                << std::endl;
      return;
  }

  // Perform RHT
  rhtVote(max_iteration, tol_distance_between_points, min_area_spanned, lidar_scan, accumulator);
  // Evaluation
  rhtEval(num_main_planes, min_vote_threshold, k_of_maxima_suppression, plane_normals,
          extracted_planes, lidar_scan, accumulator);

  visualizePlane(extracted_planes, plane_pub, tf_map_frame);

  // Give out information about extracted planes
  int color = 0;
  for (auto plane_normal : plane_normals) {
    std::cout << plane_normal[0] << " " << plane_normal[1] << " " << plane_normal[2]
              << " color: " << color % 8 << std::endl;
    ++color;
  }
}

void PlaneExtractor::iterRhtPlaneExtraction(std::vector<PointCloud<PointXYZ>> &extracted_planes,
                                            std::vector<Eigen::Vector3d> &plane_normals,
                                            PointCloud<PointXYZ> lidar_scan,
                                            std::string tf_map_frame, ros::Publisher &plane_pub) {
  std::cout << "///////////////////////////////////////////////" << std::endl;
  std::cout << "    Iterative RHT Plane Extraction started     " << std::endl;
  std::cout << "///////////////////////////////////////////////" << std::endl;

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  plane_pub = nh.advertise<sensor_msgs::PointCloud2>("extracted_planes", 1, true);

  int accumulator_choice = nh_private.param<int>("iterAccumulatorChoice", 2);
  double rho_resolution = nh_private.param<double>("iterAccumulatorRhoResolution", 0.1);
  double theta_resolution = nh_private.param<double>("iterAccumulatorThetaResolution", 0.04);
  double psi_resolution = nh_private.param<double>("iterAccumulatorPsiResolution", 0.04);
  int k_of_maxima_suppression = nh_private.param<int>("iterAccumulatorKMaxSuppress", 14);
  int min_vote_threshold = nh_private.param<int>("iterAccumulatorMinThreshold", 0);

  int number_of_iteration = nh_private.param<int>("iterRHTIter", 8);
  int iteration_per_plane = nh_private.param<int>("iterRHTIterPerIter", 10000);
  double tol_distance_between_points = nh_private.param<double>("iterRHTTolDist", 3);
  double min_area_spanned = nh_private.param<double>("iterRHTMinArea", 0.5);
  int number_of_plane_per_iter = nh_private.param<int>("iterRHTNumPlanePerIter", 1);

  // Copy lidar frame to remove points
  PointCloud<PointXYZ>::Ptr copy_lidar_scan(new PointCloud<PointXYZ>());
  copyPointCloud(lidar_scan, *copy_lidar_scan);

  PointXYZ origin(0, 0, 0);
  double max_distance_to_point = 0;
  for (auto lidar_scan_point : lidar_scan) {
    if ((double)geometry::distance(origin, lidar_scan_point) > max_distance_to_point)
      max_distance_to_point = (double)geometry::distance(origin, lidar_scan_point);
  }

  // Settings for accumulator discretization (rho, theta, psi)
  HoughAccumulator *accumulator;
  Eigen::Vector3d min_coord(0, 0, -(double)M_PI / 2);
  Eigen::Vector3d max_coord(max_distance_to_point, (double)2 * M_PI, (double)M_PI / 2);
  Eigen::Vector3d bin_size(rho_resolution, theta_resolution, psi_resolution);
  switch (accumulator_choice) {
    case 1:
      accumulator = new ArrayAccumulator(min_coord, bin_size, max_coord);
      break;
    case 2:
      accumulator = new BallAccumulator(min_coord, bin_size, max_coord);
      break;
    default:
      std::cout << "No valid accumulator to number " << accumulator_choice << ", return ..."
                << std::endl;
      return;
  }

  // Setup to remove detected planes
  ExtractIndices<PointXYZ> indices_filter;
  std::vector<std::vector<int>> inlier_ids;
  std::vector<int> rm_indices;

  // Part solution for single iterations
  std::vector<Eigen::Vector3d> iter_plane_normals;
  std::vector<PointCloud<PointXYZ>> iter_extracted_planes;

  for (int iter = 0; iter < number_of_iteration; ++iter) {
    // Perform RHT
    rhtVote(iteration_per_plane, tol_distance_between_points, min_area_spanned, *copy_lidar_scan,
            accumulator);

    // Evaluation
    inlier_ids = rhtEval(number_of_plane_per_iter, min_vote_threshold, k_of_maxima_suppression,
                         iter_plane_normals, iter_extracted_planes, *copy_lidar_scan, accumulator);

    // Add part solution to final solution
    for (int plane_nr = 0; plane_nr < iter_plane_normals.size(); ++plane_nr) {
      plane_normals.push_back(iter_plane_normals[plane_nr]);
      extracted_planes.push_back(iter_extracted_planes[plane_nr]);
      rm_indices.insert(rm_indices.end(), inlier_ids[plane_nr].begin(), inlier_ids[plane_nr].end());
    }

    // Filter out found planes
    boost::shared_ptr<std::vector<int>> inliers_ptr =
        boost::make_shared<std::vector<int>>(rm_indices);
    indices_filter.setInputCloud(copy_lidar_scan);
    indices_filter.setIndices(inliers_ptr);
    indices_filter.setNegative(true);
    indices_filter.filter(*copy_lidar_scan);
    if (copy_lidar_scan->size() == 0) break;

    // Reset for next iteration
    iter_extracted_planes.clear();
    inlier_ids.clear();
    rm_indices.clear();
    accumulator->reset();
  }

  visualizePlane(extracted_planes, plane_pub, tf_map_frame);

  // Give out information about extracted planes
  int color = 0;
  for (auto plane_normal : plane_normals) {
    std::cout << plane_normal[0] << " " << plane_normal[1] << " " << plane_normal[2]
              << " color: " << color % 8 << std::endl;
    ++color;
  }
}

void PlaneExtractor::pclPlaneExtraction(std::vector<PointCloud<PointXYZ>> &extracted_planes,
                                        std::vector<Eigen::Vector3d> &plane_normals,
                                        PointCloud<PointXYZ> lidar_scan, std::string tf_map_frame,
                                        ros::Publisher &plane_pub) {
  std::cout << "///////////////////////////////////////////////" << std::endl;
  std::cout << "      PCL RANSAC Plane Extraction started      " << std::endl;
  std::cout << "///////////////////////////////////////////////" << std::endl;

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // This code is from www.pointclouds.org/documentation/tutorials/planar_segmentation.php with some
  // modifications

  plane_pub = nh.advertise<sensor_msgs::PointCloud2>("extracted_planes", 1, true);

  double distance_threshold = nh_private.param<double>("PCLDistanceThreshold", 0.01);
  int max_number_of_plane = nh_private.param<int>("PCLMaxNumPlane", 8);
  int min_number_of_inlier = nh_private.param<int>("PCLMinInlier", 15);

  // Find plane with PCL
  PointCloud<PointXYZ>::Ptr copy_lidar_scan(new PointCloud<PointXYZ>);
  copyPointCloud(lidar_scan, *copy_lidar_scan);

  std::cout << "Setup Plane Model Extraction" << std::endl;
  ModelCoefficients::Ptr coefficients(new ModelCoefficients);
  PointIndices::Ptr inliers(new PointIndices);
  // Create the segmentation object
  SACSegmentation<PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(SACMODEL_PLANE);
  seg.setMethodType(SAC_RANSAC);
  seg.setDistanceThreshold(distance_threshold);

  std::cout << "Start to extract planes" << std::endl;

  PointCloud<PointXYZ>::Ptr extracted_inlier_points(new PointCloud<PointXYZ>);
  ExtractIndices<PointXYZ> indices_filter;
  PointXYZ normal_of_plane;
  double norm_of_normal;
  Eigen::Vector3d actuel_plane_normal;

  do {
    seg.setInputCloud(copy_lidar_scan);
    seg.segment(*inliers, *coefficients);

    // Extract inlier of actuel plane
    extracted_inlier_points->clear();
    for (auto indices : inliers->indices)
      extracted_inlier_points->push_back(copy_lidar_scan->points[indices]);

    // Add to return data
    if (extracted_inlier_points->size() > min_number_of_inlier) {
      extracted_planes.push_back(*extracted_inlier_points);

      // Read plane coefficients
      actuel_plane_normal[0] = coefficients->values[0];
      actuel_plane_normal[1] = coefficients->values[1];
      actuel_plane_normal[2] = coefficients->values[2];

      plane_normals.push_back(actuel_plane_normal);

      std::cout << "Plane found (nr. " << extracted_planes.size() << ")" << std::endl;

      indices_filter.setInputCloud(copy_lidar_scan);
      indices_filter.setIndices(inliers);
      indices_filter.setNegative(true);
      indices_filter.filter(*copy_lidar_scan);
    }
  } while (extracted_planes.size() < max_number_of_plane &&
           extracted_inlier_points->size() > min_number_of_inlier &&
           copy_lidar_scan->size() > min_number_of_inlier);

  // Visualize plane
  visualizePlane(extracted_planes, plane_pub, tf_map_frame);

  // Give out information about extracted planes
  int color = 0;
  for (auto plane_normal : plane_normals) {
    std::cout << plane_normal[0] << " " << plane_normal[1] << " " << plane_normal[2]
              << " color: " << color % 8 << std::endl;
    ++color;
  }
}

void PlaneExtractor::cgalRegionGrowing(
    std::vector<pcl::PointCloud<pcl::PointXYZ>> &extracted_planes,
    std::vector<Eigen::Vector3d> &plane_normals, PointCloud<pcl::PointXYZ> lidar_scan,
    std::string tf_map_frame, ros::Publisher &plane_pub) {
  std::cout << "///////////////////////////////////////////////" << std::endl;
  std::cout << "         CGAL Region Growing started           " << std::endl;
  std::cout << "///////////////////////////////////////////////" << std::endl;

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  plane_pub = nh.advertise<sensor_msgs::PointCloud2>("extracted_planes", 1, true);

  // This code is from cpt_deviation_analysis/deviations.cpp/Deviations::runShapeDetection with some
  // modifications

  // https://doc.cgal.org/4.13.1/Point_set_shape_detection_3/index.html#title7

  cgal::Pwn_vector points;  // Points with normals.

  // load points from pcl cloud
  for (auto point : lidar_scan.points) {
    cgal::Point_with_normal pwn;
    pwn.first = cgal::ShapeKernel::Point_3(point.x, point.y, point.z);
    points.push_back(pwn);
  }

  // Estimate normals direction
  const int nb_neighbors = 20;
  CGAL::pca_estimate_normals<CGAL::Parallel_tag>(
      points, nb_neighbors,
      CGAL::parameters::point_map(cgal::Point_map()).normal_map(cgal::Normal_map()));

  // Instantiate shape detection engine.
  cgal::Region_growing shape_detection;
  shape_detection.set_input(points);
  // Registers planar shapes via template method (could also register other shapes)
  shape_detection.template add_shape_factory<cgal::ShapePlane>();
  // Build internal data structures.
  shape_detection.preprocess();

  // Sets parameters for shape detection.
  cgal::Region_growing::Parameters parameters;

  // Detect shapes with at least X points.
  parameters.min_points = nh_private.param<int>("CGALRegionGrowMinNumInliers", 80);
  // Sets maximum Euclidean distance between a point and a shape.
  parameters.epsilon = nh_private.param<float>("CGALRegionGrowMaxDistToPlane", 0.07);
  // Sets maximum Euclidean distance between points to be clustered.
  parameters.cluster_epsilon = nh_private.param<float>("CGALRegionGrowMaxDistBetwPoint", 10);
  // Sets maximum normal deviation.
  parameters.normal_threshold =
      nh_private.param<float>("CGALRegionGrowMaxDiffNormalThreshold", 0.4);

  // Detect registered shapes with parameters
  shape_detection.detect(parameters);

  // Compute coverage, i.e. ratio of the points assigned to a shape
  cgal::FT coverage = cgal::FT(points.size() - shape_detection.number_of_unassigned_points()) /
                      cgal::FT(points.size());

  // Prints number of assigned shapes and unassigned points
  std::cout << shape_detection.shapes().end() - shape_detection.shapes().begin() << " primitives, "
            << coverage << " coverage" << std::endl;

  // Take result
  cgal::Region_growing::Plane_range shapes = shape_detection.planes();

  // Apply Regularization
  if (nh_private.param<bool>("CGALRGRegulActive", false)) {
    std::cout << "Start Regularization ..." << std::endl;
    CGAL::regularize_planes(
        points, cgal::Point_map(), shapes, CGAL::Shape_detection_3::Plane_map<cgal::Traits>(),
        CGAL::Shape_detection_3::Point_to_shape_index_map<cgal::Traits>(points, shapes),
        nh_private.param<bool>("CGALRGRegulParall", false),    // regularize parallelism
        nh_private.param<bool>("CGALRGRegulOrthog", false),    // regularize orthogonality,
        nh_private.param<bool>("CGALRGRegulCoplanar", false),  // regularize coplanarity
        false,                                                 // regularize Z-symmetry (default)
        nh_private.param<float>("CGALRGRegulParallOrthTol",
                                10),  // tolerance of parallelism / orthogonality
        nh_private.param<float>("CGALRGRegulCoplanarTol",
                                0.5));  // tolerance of coplanarity
  }

  // Characterize shapes
  cgal::Region_growing::Plane_range::iterator shapeIt = shapes.begin();
  PointCloud<PointXYZ> actual_plane_inlier;
  while (shapeIt != shapes.end()) {
    cgal::ShapePlane *plane = dynamic_cast<cgal::ShapePlane *>(shapeIt->get());
    cgal::ShapeKernel::Vector_3 normal = plane->plane_normal();

    plane_normals.push_back(Eigen::Vector3d(normal.x(), normal.y(), normal.z()));

    // Iterates through point indices assigned to each detected shape
    std::vector<std::size_t>::const_iterator index_it =
        (*shapeIt)->indices_of_assigned_points().begin();
    actual_plane_inlier.clear();
    while (index_it != (*shapeIt)->indices_of_assigned_points().end()) {
      // Retrieve point
      const cgal::Point_with_normal &p = *(points.begin() + (*index_it));

      // Add point to inliers of plane
      PointXYZ pc_point(p.first.x(), p.first.y(), p.first.z());
      actual_plane_inlier.push_back(pc_point);
      // Proceeds with next point
      index_it++;
    }
    extracted_planes.push_back(actual_plane_inlier);
    shapeIt++;
  }

  // Give out information about extracted planes
  int color = 0;
  for (auto plane_normal : plane_normals) {
    std::cout << plane_normal[0] << " " << plane_normal[1] << " " << plane_normal[2]
              << " color: " << color % 8 << std::endl;
    ++color;
  }
  visualizePlane(extracted_planes, plane_pub, tf_map_frame);
};

// Helper functions
void PlaneExtractor::visualizePlane(std::vector<PointCloud<PointXYZ>> &extracted_planes,
                                    ros::Publisher &plane_pub, std::string tf_map_frame) {
  std::cout << "Start Visualization" << std::endl;

  int color[8][3] = {{0, 0, 0},     {255, 0, 0},   {0, 255, 0},   {0, 0, 255},
                     {255, 255, 0}, {255, 0, 255}, {0, 255, 255}, {255, 255, 255}};

  PointCloud<PointXYZRGB>::Ptr segmented_point_cloud(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr colored_inlier_points(new PointCloud<PointXYZRGB>);
  // Create segmented point cloud
  if (extracted_planes.size() == 0) {
    std::cout << "No planes to visualize" << std::endl;
    return;
  } else {
    std::cout << "Found " << extracted_planes.size() << " planes, visualize plane inliers... "
              << std::endl;
    for (std::size_t i = 0; i < extracted_planes.size(); ++i) {
      colored_inlier_points->clear();
      copyPointCloud(extracted_planes[i], *colored_inlier_points);
      for (std::size_t j = 0; j < colored_inlier_points->size(); ++j) {
        colored_inlier_points->points[j].r = color[i % 8][0];
        colored_inlier_points->points[j].g = color[i % 8][1];
        colored_inlier_points->points[j].b = color[i % 8][2];
        colored_inlier_points->points[j].a = 255;
      }
      *segmented_point_cloud += *colored_inlier_points;
    }
  }
  // Publish segmented point cloud
  sensor_msgs::PointCloud2 segmentation_mesg;
  segmented_point_cloud->header.frame_id = tf_map_frame;
  toROSMsg(*segmented_point_cloud, segmentation_mesg);
  plane_pub.publish(segmentation_mesg);
  std::cout << "Publish plane segmentation" << std::endl;
}

void PlaneExtractor::rhtVote(int max_iteration, double tol_distance_between_points,
                             double min_area_spanned, PointCloud<PointXYZ> lidar_scan,
                             HoughAccumulator *accumulator) {
  // Setup needed variables
  int reference_point_ids[3];
  PointXYZ sampled_point[3];
  Eigen::Vector3d reference_point[3];
  Eigen::Vector3d normal_of_plane;
  double norm_of_normal;
  Eigen::Vector3d vote;

  int pointcloud_size = lidar_scan.size();

  std::cout << "Start voting" << std::endl;
  for (int i = 0; i < max_iteration; ++i) {
    // Sample random points
    reference_point_ids[0] = (rand() % (pointcloud_size + 1));
    reference_point_ids[1] = (rand() % (pointcloud_size + 1));
    reference_point_ids[2] = (rand() % (pointcloud_size + 1));
    sampled_point[0] = lidar_scan.points[reference_point_ids[0]];
    sampled_point[1] = lidar_scan.points[reference_point_ids[1]];
    sampled_point[2] = lidar_scan.points[reference_point_ids[2]];

    reference_point[0] =
        Eigen::Vector3d(sampled_point[0].x, sampled_point[0].y, sampled_point[0].z);
    reference_point[1] =
        Eigen::Vector3d(sampled_point[1].x, sampled_point[1].y, sampled_point[1].z);
    reference_point[2] =
        Eigen::Vector3d(sampled_point[2].x, sampled_point[2].y, sampled_point[2].z);

    // Check if points are close
    if ((reference_point[0] - reference_point[1]).norm() > tol_distance_between_points ||
        (reference_point[1] - reference_point[2]).norm() > tol_distance_between_points ||
        (reference_point[0] - reference_point[2]).norm() > tol_distance_between_points) {
      continue;
    }

    normal_of_plane =
        (reference_point[0] - reference_point[1]).cross(reference_point[0] - reference_point[2]);
    norm_of_normal = normal_of_plane.norm();

    // Check if points are collinear or the same
    if (norm_of_normal < min_area_spanned) {
      continue;
    }

    vote(0) = normal_of_plane.dot(reference_point[0]) / norm_of_normal;  // rho
    if (vote(0) < 0) {
      vote(0) = -vote(0);
      normal_of_plane = -normal_of_plane;
    }
    vote(1) = atan2(normal_of_plane[1], normal_of_plane[0]);  // theta
    vote(2) = atan2(normal_of_plane[2],
                    sqrt(pow(normal_of_plane[0], 2) + pow(normal_of_plane[1], 2)));  // psi

    accumulator->vote(vote, reference_point_ids);
  }
  std::cout << "Voting finished" << std::endl;
}

std::vector<std::vector<int>> PlaneExtractor::rhtEval(
    int num_main_planes, int min_vote_threshold, int k_of_maxima_suppression,
    std::vector<Eigen::Vector3d> &plane_normals,
    std::vector<PointCloud<PointXYZ>> &extracted_planes, PointCloud<PointXYZ> lidar_scan,
    HoughAccumulator *accumulator) {
  std::vector<std::vector<int>> inlier_ids;
  PointCloud<PointXYZ> used_inliers;
  bool bit_point_cloud[lidar_scan.size()] = {false};

  // Evaluate voting (select between thresholding or number of extracted planes)
  std::vector<Eigen::Vector3d> plane_coefficients;
  if (num_main_planes == 0) {
    accumulator->findMaxima(min_vote_threshold, plane_coefficients, inlier_ids,
                            k_of_maxima_suppression);
  } else {
    accumulator->findMaximumPlane(num_main_planes, plane_coefficients, inlier_ids,
                                  k_of_maxima_suppression);
  }

  // Convert plane coefficients to normal
  plane_normals.clear();
  Eigen::Vector3d actual_normal;
  for (auto plane_coefficient : plane_coefficients) {
    actual_normal[0] = cos(plane_coefficient[1]) * cos(plane_coefficient[2]);
    actual_normal[1] = sin(plane_coefficient[1]) * cos(plane_coefficient[2]);
    actual_normal[2] = sin(plane_coefficient[2]);
    plane_normals.push_back(actual_normal);
  }

  // Extract inliers
  std::vector<int> rm_inlier_ids;
  for (int plane_nr = 0; plane_nr < inlier_ids.size(); ++plane_nr) {
    for (int i = 0; i < inlier_ids[plane_nr].size(); ++i) {
      // make sure each point is only included once
      if (!bit_point_cloud[inlier_ids[plane_nr][i]]) {
        used_inliers.push_back(lidar_scan.points[inlier_ids[plane_nr][i]]);
        bit_point_cloud[inlier_ids[plane_nr][i]] = true;
      } else {
        // Add duplicates to removal list
        rm_inlier_ids.push_back(i);
      }
    }
    // Remove duplicates
    for (auto rm_ids = rm_inlier_ids.rbegin(); rm_ids != rm_inlier_ids.rend(); ++rm_ids) {
      inlier_ids[plane_nr].erase(inlier_ids[plane_nr].begin() + *rm_ids);
    }
    extracted_planes.push_back(used_inliers);
    used_inliers.clear();
    rm_inlier_ids.clear();
  }

  return inlier_ids;
}
}  // namespace matching_algorithms
}  // namespace cad_percept
