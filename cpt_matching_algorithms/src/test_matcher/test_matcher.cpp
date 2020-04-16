#include "test_matcher/test_matcher.h"

namespace cad_percept {
namespace matching_algorithms {

TestMatcher::TestMatcher(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
  /*//////////////////////////////////////
                 Setup
  ///////////////////////////////////////*/
  // Get Parameters from Server
  cad_topic_ = nh_private_.param<std::string>("cadTopic", "fail");
  lidar_topic_ = nh_private_.param<std::string>("lidarTopic", "fail");
  sim_lidar_topic_ = nh_private_.param<std::string>("simlidarTopic", "fail");
  ground_truth_topic_ = nh_private_.param<std::string>("groundtruthTopic", "fail");
  input_queue_size_ = nh_private_.param<int>("inputQueueSize", 10);
  tf_map_frame_ = nh_private_.param<std::string>("tfMapFrame", "/map");
  tf_lidar_frame_ = nh_private_.param<std::string>("tfLidarFrame", "/marker_pose");
  use_sim_lidar_ = nh_private_.param<bool>("usetoyproblem", false);

  // Get Subscriber
  map_sub_ = nh_.subscribe(cad_topic_, input_queue_size_, &TestMatcher::getCAD, this);
  lidar_sub_ = nh_.subscribe(lidar_topic_, input_queue_size_, &TestMatcher::getLidar, this);
  lidar_sim_sub_ =
      nh_.subscribe(sim_lidar_topic_, input_queue_size_, &TestMatcher::getSimLidar, this);
  gt_sub_ =
      nh_.subscribe(ground_truth_topic_, input_queue_size_, &TestMatcher::getGroundTruth, this);

  // Get Publisher
  scan_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("matched_point_cloud", input_queue_size_, true);
  sample_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("sample_map", input_queue_size_, true);
}

// Get CAD and sample points
void TestMatcher::getCAD(const cgal_msgs::TriangleMeshStamped& cad_mesh_in) {
  if (!map_ready_) {
    std::cout << "Processing CAD mesh" << std::endl;
    std::string frame_id = cad_mesh_in.header.frame_id;
    cad_percept::cgal::msgToMeshModel(cad_mesh_in.mesh, &reference_mesh_);

    // Get transformation from /map to mesh
    tf::StampedTransform transform;
    tf::TransformListener tf_listener_(ros::Duration(30));
    try {
      tf_listener_.waitForTransform(tf_map_frame_, frame_id, ros::Time(0), ros::Duration(5.0));
      tf_listener_.lookupTransform(tf_map_frame_, frame_id, ros::Time(0),
                                   transform);  // get transformation at latest time T_map_to_frame
    } catch (tf::TransformException ex) {
      ROS_ERROR_STREAM("Couldn't find transformation to mesh system");
    }

    cad_percept::cgal::Transformation ctransformation;
    cgal::tfTransformationToCGALTransformation(transform, ctransformation);
    reference_mesh_->transform(ctransformation);

    // Sample from mesh
    sample_map_.clear();
    sample_density_ = nh_private_.param<int>("mapSamplingDensity", 100);
    int n_points = reference_mesh_->getArea() * sample_density_;
    cad_percept::cpt_utils::sample_pc_from_mesh(reference_mesh_->getMesh(), n_points, 0.0,
                                                &sample_map_);

    // Extract planes
    // load_example();
    std::unordered_map<std::string, std::string> facetToPlane;
    std::unordered_multimap<std::string, std::string> planeToFacets;
    // If I set eps to a very large number, shouldn't we then get only one plane?
    reference_mesh_->findAllCoplanarFacets(&facetToPlane, &planeToFacets, 10);
    std::cout << "Extracted coplanar Facets" << std::endl;

    std::multimap<std::string, std::string> grouped_planeToFacets(planeToFacets.begin(),
                                                                  planeToFacets.end());

    cgal::Plane actual_plane;
    cgal::Triangle actual_facet;
    cgal::Vector actual_facet_vertex[3];
    Eigen::Vector3d point_on_plane;
    Eigen::Vector3d normal_of_plane;
    pcl::PointNormal norm_point;
    float map_plane_area_threshold = 5;
    float max_facet_size;
    std::string previous_plane_id = "-1";

    for (auto facet_of_plane : grouped_planeToFacets) {
      // Search for largest facet of a plane
      if (facet_of_plane.first.compare(previous_plane_id) && previous_plane_id.compare("-1")) {
        // Reset max size and add best desription of previous plane if a new plane id appears
        std::cout << "added plane from mesh nr " << previous_plane_id << std::endl;
        std::cout << "point on plane " << norm_point.x << " " << norm_point.y << " " << norm_point.z
                  << std::endl;
        std::cout << "normal of plane " << norm_point.normal_x << " " << norm_point.normal_y << " "
                  << norm_point.normal_z << std::endl;

        map_planes_.push_back(norm_point);
        max_facet_size = 0;
      }
      // Take largest facet to describe plane
      if (reference_mesh_->getArea(reference_mesh_->getFacetHandleFromId(facet_of_plane.second)) >
          std::max(map_plane_area_threshold, max_facet_size)) {
        actual_facet = reference_mesh_->getTriangle(facet_of_plane.second);
        actual_plane = actual_facet.supporting_plane();
        actual_facet_vertex[0] = cgal::Vector(
            actual_facet.vertex(0).x(), actual_facet.vertex(0).y(), actual_facet.vertex(0).z());
        actual_facet_vertex[1] = cgal::Vector(
            actual_facet.vertex(1).x(), actual_facet.vertex(1).y(), actual_facet.vertex(1).z());
        actual_facet_vertex[2] = cgal::Vector(
            actual_facet.vertex(2).x(), actual_facet.vertex(2).y(), actual_facet.vertex(2).z());

        point_on_plane = cgal::cgalVectorToEigenVector(
            (actual_facet_vertex[0] + actual_facet_vertex[1] + actual_facet_vertex[2]) /
            3);  // mean of vertices of facet
        normal_of_plane =
            cgal::cgalVectorToEigenVector(actual_plane.orthogonal_vector()).normalized();

        norm_point.x = point_on_plane(0);
        norm_point.y = point_on_plane(1);
        norm_point.z = point_on_plane(2);
        norm_point.normal_x = normal_of_plane(0);
        norm_point.normal_y = normal_of_plane(1);
        norm_point.normal_z = normal_of_plane(2);

        // direct normal away from origin
        if ((norm_point.x * norm_point.normal_x + norm_point.y * norm_point.normal_y +
             norm_point.z * norm_point.normal_z) < 0) {
          norm_point.normal_x = -norm_point.normal_x;
          norm_point.normal_y = -norm_point.normal_y;
          norm_point.normal_z = -norm_point.normal_z;
        }

        max_facet_size =
            reference_mesh_->getArea(reference_mesh_->getFacetHandleFromId(facet_of_plane.second));
      }
      previous_plane_id = facet_of_plane.first;
    }
    std::cout << "Found " << map_planes_.size() << " planes in the map " << std::endl;

    std::cout << "CAD ready" << std::endl;
    map_ready_ = true;

    if (lidar_scan_ready_) {
      match();
      if (ground_truth_ready_) {
        evaluate();
      }
    }
  }
}

// Get real LiDAR data
void TestMatcher::getLidar(const sensor_msgs::PointCloud2& lidar_scan_p2) {
  if (!lidar_scan_ready_ && !use_sim_lidar_) {
    std::cout << "Processing lidar frame" << std::endl;

    // Convert PointCloud2 to PointCloud
    pcl::PCLPointCloud2 lidar_pc2;
    pcl_conversions::toPCL(lidar_scan_p2, lidar_pc2);
    pcl::fromPCLPointCloud2(lidar_pc2, lidar_scan_);
    std::vector<int> nan_indices;
    pcl::removeNaNFromPointCloud(lidar_scan_, lidar_scan_, nan_indices);
    if (nan_indices.size() != 0) {
      std::cout << "Attention: Detected NaNs in the given point cloud. Removed this values..."
                << std::endl;
    }

    std::cout << "Lidar scan ready" << std::endl;
    lidar_scan_ready_ = true;

    // Get static structure information point cloud
    if (nh_private_.param<bool>("useStructureFilter", false)) {
      pcl::fromROSMsg(lidar_scan_p2, static_structure_cloud_);
      pcl::removeNaNFromPointCloud(static_structure_cloud_, static_structure_cloud_, nan_indices);
    }

    if (map_ready_) {
      match();
      if (ground_truth_ready_) {
        evaluate();
      }
    }
  }
}

// Get real ground truth position
void TestMatcher::getGroundTruth(const geometry_msgs::PointStamped& gt_in) {
  if (!ground_truth_ready_ && !use_sim_lidar_) {
    ground_truth_ = gt_in;
    // Transform gt into map frame
    ground_truth_.point.x = -ground_truth_.point.x;
    ground_truth_.point.y = -ground_truth_.point.y;
    std::cout << "Got ground truth data" << std::endl;
    ground_truth_ready_ = true;

    if (ready_for_eval_) {
      evaluate();
    }
  }
}

// Get LiDAR data and ground truth from simulator
void TestMatcher::getSimLidar(const sensor_msgs::PointCloud2& lidar_scan_p2) {
  if (!lidar_scan_ready_ && use_sim_lidar_) {
    std::cout << "Processing lidar frame" << std::endl;

    // Convert PointCloud2 to PointCloud
    pcl::PCLPointCloud2 lidar_pc2;
    pcl_conversions::toPCL(lidar_scan_p2, lidar_pc2);
    pcl::fromPCLPointCloud2(lidar_pc2, lidar_scan_);

    std::cout << "Simulated Lidar frame ready" << std::endl;

    tf::StampedTransform transform;
    tf::TransformListener tf_listener(ros::Duration(10));
    try {
      tf_listener.waitForTransform(tf_map_frame_, tf_lidar_frame_, ros::Time(0),
                                   ros::Duration(5.0));
      tf_listener.lookupTransform(tf_map_frame_, tf_lidar_frame_, ros::Time(0), transform);
    } catch (tf::TransformException ex) {
      ROS_INFO_STREAM("Couldn't find transformation to lidar frame");
      return;
    }
    cad_percept::cgal::Transformation ctransformation;
    cgal::tfTransformationToCGALTransformation(transform, ctransformation);
    Eigen::Matrix4d etransformation;
    cgal::cgalTransformationToEigenTransformation(ctransformation, &etransformation);
    Eigen::Matrix3d erotation = etransformation.block(0, 0, 3, 3);
    Eigen::Quaterniond q(erotation);

    // Ground truth is T_map,lidar
    ground_truth_.point.x = etransformation(0, 3);
    ground_truth_.point.y = etransformation(1, 3);
    ground_truth_.point.z = etransformation(2, 3);
    gt_quat_.push_back(q.w());
    gt_quat_.push_back(q.x());
    gt_quat_.push_back(q.y());
    gt_quat_.push_back(q.z());

    std::cout << "Got simulated ground truth data" << std::endl;

    lidar_scan_ready_ = true;
    ground_truth_ready_ = true;

    if (map_ready_) {
      match();
      evaluate();
    }
  }
}

// Match clouds and evaluate
void TestMatcher::match() {
  std::cout << "Received LiDAR and CAD as point cloud" << std::endl;

  /*//////////////////////////////////////
                 Matching
  ///////////////////////////////////////*/

  // Selection of mapper
  std::string matcher = nh_private_.param<std::string>("Matcher", "fail");
  if (!matcher.compare("template")) {
    templateMatch();
  } else if (!matcher.compare("GoICP")) {
    goicpMatch();
  } else if (!matcher.compare("PlaneMatcher")) {
    // Filtering / Preprocessing Point Cloud
    if (nh_private_.param<bool>("useStructureFilter", false)) {
      int structure_threshold = nh_private_.param<int>("StructureThreshold", 150);
      CloudFilter::filterStaticObject(structure_threshold, lidar_scan_, static_structure_cloud_);
    }
    if (nh_private_.param<bool>("useVoxelCentroidFilter", false)) {
      float search_radius = nh_private_.param<float>("Voxelsearchradius", 0.01);
      CloudFilter::filterVoxelCentroid(search_radius, lidar_scan_);
    }
    // Plane Extraction
    std::vector<pcl::PointCloud<pcl::PointXYZ>> extracted_planes;
    std::vector<std::vector<double>> plane_coefficients;
    std::string extractor = nh_private_.param<std::string>("PlaneExtractor", "fail");
    if (!extractor.compare("pclPlaneExtraction")) {
      PlaneExtractor::pclPlaneExtraction(extracted_planes, plane_coefficients, lidar_scan_,
                                         tf_map_frame_, plane_pub_);
    } else if (!extractor.compare("rhtPlaneExtraction")) {
      PlaneExtractor::rhtPlaneExtraction(extracted_planes, plane_coefficients, lidar_scan_,
                                         tf_map_frame_, plane_pub_);
    } else if (!extractor.compare("iterRhtPlaneExtraction")) {
      PlaneExtractor::iterRhtPlaneExtraction(extracted_planes, plane_coefficients, lidar_scan_,
                                             tf_map_frame_, plane_pub_);

    } else {
      std::cout << "Error: Could not find given plane extractor" << std::endl;
    }

    // Convert plane coefficients back to normal (could be done easily in the function itself)
    pcl::PointNormal norm_point;
    pcl::PointXYZ plane_centroid;
    int plane_nr = 0;
    for (auto plane_coefficient : plane_coefficients) {
      pcl::computeCentroid(extracted_planes[plane_nr], plane_centroid);
      norm_point.x = plane_centroid.x;
      norm_point.y = plane_centroid.y;
      norm_point.z = plane_centroid.z;

      norm_point.normal_x = std::cos(plane_coefficient[1]) * std::cos(plane_coefficient[2]);
      norm_point.normal_y = std::sin(plane_coefficient[1]) * std::cos(plane_coefficient[2]);
      norm_point.normal_z = std::sin(plane_coefficient[2]);
      scan_planes_.push_back(norm_point);
      plane_nr++;
    }
    std::string plane_matcher = nh_private_.param<std::string>("PlaneMatch", "fail");
    // Plane Matching (Get T_map,lidar)
    if (!plane_matcher.compare("IntersectionPatternMatcher")) {
      PlaneMatch::IntersectionPatternMatcher(transform_TR_, scan_planes_, map_planes_,
                                             room_boundaries);
    } else if (!plane_matcher.compare("useMatchSolution")) {
      PlaneMatch::loadExampleSol(transform_TR_, scan_planes_, map_planes_);
    } else if (!plane_matcher.compare("LineSegmentRansac")) {
      PlaneMatch::LineSegmentRansac(transform_TR_, scan_planes_, map_planes_, room_boundaries);
    } else {
      std::cout << "Error: Could not find given plane matcher" << std::endl;
    }
  } else {
    std::cout << "Error: Could not find given matcher" << std::endl;
  }

  /*//////////////////////////////////////
                Transformation
  ///////////////////////////////////////*/

  // Transform LiDAR frame
  Eigen::Matrix4f res_transform = Eigen::Matrix4f::Identity();
  Eigen::Vector3f translation(transform_TR_[0], transform_TR_[1], transform_TR_[2]);
  Eigen::Quaternionf q(transform_TR_[3], transform_TR_[4], transform_TR_[5], transform_TR_[6]);
  res_transform.block(0, 0, 3, 3) = q.matrix();
  res_transform.block(0, 3, 3, 1) = translation;

  pcl::transformPointCloud(lidar_scan_, lidar_scan_, res_transform.inverse());

  /*//////////////////////////////////////
                Visualization
  ///////////////////////////////////////*/
  DP ref_sample_map = cad_percept::cpt_utils::pointCloudToDP(sample_map_);
  sample_map_pub_.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(
      ref_sample_map, tf_map_frame_, ros::Time::now()));

  DP ref_dp = cpt_utils::pointCloudToDP(lidar_scan_);
  scan_pub_.publish(
      PointMatcher_ros::pointMatcherCloudToRosMsg<float>(ref_dp, tf_map_frame_, ros::Time::now()));
  ready_for_eval_ = true;
}  // namespace matching_algorithms

void TestMatcher::evaluate() {
  /*//////////////////////////////////////
                Evaluation
  ///////////////////////////////////////*/

  std::cout << "calculated position: x: " << transform_TR_[0] << " y: " << transform_TR_[1]
            << " z: " << transform_TR_[2] << std::endl;
  std::cout << "ground truth position: x: " << ground_truth_.point.x
            << " y: " << ground_truth_.point.y << " z: " << ground_truth_.point.z << std::endl;
  std::cout << "calculated orientation: qw: " << transform_TR_[3] << " qx: " << transform_TR_[4]
            << " qy: " << transform_TR_[5] << " qz: " << transform_TR_[6] << std::endl;

  if (use_sim_lidar_) {
    std::cout << "ground truth orientation: qw: " << gt_quat_[0] << " qx: " << gt_quat_[1]
              << " qy: " << gt_quat_[2] << " qz: " << gt_quat_[3] << std::endl;
  }

  float error = sqrt(pow(transform_TR_[0] - ground_truth_.point.x, 2) +
                     pow(transform_TR_[1] - ground_truth_.point.y, 2) +
                     pow(transform_TR_[2] - ground_truth_.point.z, 2));
  getError(sample_map_, lidar_scan_);
  std::cout << "error (euclidean distance of translation): " << error << std::endl;
}

// Get RMSE of two point clouds
void TestMatcher::getError(PointCloud p1, PointCloud p2) {
  DP p1_dp = cpt_utils::pointCloudToDP(p1);
  DP p2_dp = cpt_utils::pointCloudToDP(p2);

  // Calculation of Hausdorff Distance withot oulier removal, source: cpt_selective_icp::mapper
  const int knn = 1;  // first closest point
  PM::Parameters params;
  params["knn"] = PointMatcherSupport::toParam(knn);
  params["epsilon"] = PointMatcherSupport::toParam(0);
  // other parameters possible
  std::shared_ptr<PM::Matcher> matcher = PM::get().MatcherRegistrar.create("KDTreeMatcher", params);

  matcher->init(p1_dp);
  PM::Matches matches = matcher->findClosests(p2_dp);
  float max_dist_1 = matches.getDistsQuantile(1.0);
  float max_dist_robust_1 = matches.getDistsQuantile(0.85);

  matcher->init(p2_dp);
  matches = matcher->findClosests(p1_dp);
  float max_dist_2 = matches.getDistsQuantile(1.0);
  float max_dist_robust_2 = matches.getDistsQuantile(0.85);

  float haussdorff_dist = std::max(max_dist_1, max_dist_2);
  float haussdorff_quantile_dist = std::max(max_dist_robust_1, max_dist_robust_2);
  std::cout << "Haussdorff distance: " << std::sqrt(haussdorff_dist) << " m" << std::endl;
  std::cout << "Haussdorff quantile distance: " << std::sqrt(haussdorff_quantile_dist) << " m"
            << std::endl;
}

void TestMatcher::load_example() {
  // normals for used map
  pcl::PointNormal norm_point;
  norm_point.x = -3.8;
  norm_point.y = -0.2;
  norm_point.z = 0;
  norm_point.normal_x = 0;
  norm_point.normal_y = 0;
  norm_point.normal_z = -1;
  map_planes_.push_back(norm_point);
  norm_point.x = 0;
  norm_point.y = 3.5;
  norm_point.z = 1;
  norm_point.normal_x = -1;
  norm_point.normal_y = 0;
  norm_point.normal_z = 0;
  map_planes_.push_back(norm_point);
  norm_point.x = 10.3;
  norm_point.y = 6.4;
  norm_point.z = 1;
  norm_point.normal_x = 0;
  norm_point.normal_y = 1;
  norm_point.normal_z = 0;
  map_planes_.push_back(norm_point);
  norm_point.x = 19.8;
  norm_point.y = 7.1;
  norm_point.z = 1;
  norm_point.normal_x = -1;
  norm_point.normal_y = 0;
  norm_point.normal_z = 0;
  map_planes_.push_back(norm_point);
  norm_point.x = 27.5;
  norm_point.y = 7.6;
  norm_point.z = 1;
  norm_point.normal_x = 0;
  norm_point.normal_y = 1;
  norm_point.normal_z = 0;
  map_planes_.push_back(norm_point);
  norm_point.x = 34.3;
  norm_point.y = 3.8;
  norm_point.z = 1;
  norm_point.normal_x = 1;
  norm_point.normal_y = 0;
  norm_point.normal_z = 0;
  map_planes_.push_back(norm_point);
  norm_point.x = 36.15;
  norm_point.y = 0;
  norm_point.z = 1;
  norm_point.normal_x = 0;
  norm_point.normal_y = 1;
  norm_point.normal_z = 0;
  map_planes_.push_back(norm_point);
  norm_point.x = 37.9;
  norm_point.y = -1;
  norm_point.z = 1;
  norm_point.normal_x = 1;
  norm_point.normal_y = 0;
  norm_point.normal_z = 0;
  map_planes_.push_back(norm_point);
  norm_point.x = 36.15;
  norm_point.y = -2.6;
  norm_point.z = 1;
  norm_point.normal_x = 0;
  norm_point.normal_y = -1;
  norm_point.normal_z = 0;
  map_planes_.push_back(norm_point);
  norm_point.x = 34.4;
  norm_point.y = -4.16;
  norm_point.z = 1;
  norm_point.normal_x = 1;
  norm_point.normal_y = 0;
  norm_point.normal_z = 0;
  map_planes_.push_back(norm_point);
  norm_point.x = 37.65;
  norm_point.y = -6.3;
  norm_point.z = 1;
  norm_point.normal_x = 0;
  norm_point.normal_y = -1;
  norm_point.normal_z = 0;
  map_planes_.push_back(norm_point);
  norm_point.x = 29.8;
  norm_point.y = -7.24;
  norm_point.z = 1;
  norm_point.normal_x = 1;
  norm_point.normal_y = 0;
  norm_point.normal_z = 0;
  map_planes_.push_back(norm_point);
  norm_point.x = 26.3;
  norm_point.y = -8;
  norm_point.z = 1;
  norm_point.normal_x = 0;
  norm_point.normal_y = -1;
  norm_point.normal_z = 0;
  map_planes_.push_back(norm_point);
  norm_point.x = 21.6;
  norm_point.y = -7.1;
  norm_point.z = 1;
  norm_point.normal_x = -1;
  norm_point.normal_y = 0;
  norm_point.normal_z = 0;
  map_planes_.push_back(norm_point);
  norm_point.x = 9.74;
  norm_point.y = -6;
  norm_point.z = 1;
  norm_point.normal_x = 0;
  norm_point.normal_y = -1;
  norm_point.normal_z = 0;
  map_planes_.push_back(norm_point);
  norm_point.x = -1.86;
  norm_point.y = -7.1;
  norm_point.z = 1;
  norm_point.normal_x = 1;
  norm_point.normal_y = 0;
  norm_point.normal_z = 0;
  map_planes_.push_back(norm_point);
  norm_point.x = -3.1;
  norm_point.y = -7.16;
  norm_point.z = 1;
  norm_point.normal_x = -1;
  norm_point.normal_y = 0;
  norm_point.normal_z = 0;
  map_planes_.push_back(norm_point);
  norm_point.x = -11.7;
  norm_point.y = -6.22;
  norm_point.z = 1;
  norm_point.normal_x = 0;
  norm_point.normal_y = -1;
  norm_point.normal_z = 0;
  map_planes_.push_back(norm_point);
  norm_point.x = -18.5;
  norm_point.y = -3.95;
  norm_point.z = 1;
  norm_point.normal_x = -1;
  norm_point.normal_y = 0;
  norm_point.normal_z = 0;
  map_planes_.push_back(norm_point);
  norm_point.x = -20.97;
  norm_point.y = -2.5;
  norm_point.z = 1;
  norm_point.normal_x = 0;
  norm_point.normal_y = -1;
  norm_point.normal_z = 0;
  map_planes_.push_back(norm_point);
  norm_point.x = -24;
  norm_point.y = -1.3;
  norm_point.z = 1;
  norm_point.normal_x = -1;
  norm_point.normal_y = 0;
  norm_point.normal_z = 0;
  map_planes_.push_back(norm_point);
  norm_point.x = -11.6;
  norm_point.y = -0;
  norm_point.z = 1;
  norm_point.normal_x = 0;
  norm_point.normal_y = 1;
  norm_point.normal_z = 0;
  map_planes_.push_back(norm_point);

  // load map boundaries
  room_boundaries(1, 0) = 0;
  room_boundaries(1, 1) = 6.4;
  room_boundaries(2, 0) = 0;
  room_boundaries(2, 1) = 19.8;
  room_boundaries(3, 0) = 6.4;
  room_boundaries(3, 1) = 7.6;
  room_boundaries(4, 0) = 19.8;
  room_boundaries(4, 1) = 34.3;
  room_boundaries(5, 0) = 0;
  room_boundaries(5, 1) = 7.6;
  room_boundaries(6, 0) = 34.3;
  room_boundaries(6, 1) = 37.9;
  room_boundaries(7, 0) = -2.6;
  room_boundaries(7, 1) = 0;
  room_boundaries(8, 0) = 34.4;
  room_boundaries(8, 1) = 37.9;
  room_boundaries(9, 0) = -6.3;
  room_boundaries(9, 1) = -2.6;
  room_boundaries(10, 0) = 29.8;
  room_boundaries(10, 1) = 36.15;
  room_boundaries(11, 0) = -8;
  room_boundaries(11, 1) = -6.3;
  room_boundaries(12, 0) = 21.6;
  room_boundaries(12, 1) = 29.8;
  room_boundaries(13, 0) = -8;
  room_boundaries(13, 1) = -6.3;
  room_boundaries(14, 0) = -1.86;
  room_boundaries(14, 1) = 21.6;
  room_boundaries(15, 0) = -100;
  room_boundaries(15, 1) = -6;
  room_boundaries(16, 0) = -100;
  room_boundaries(16, 1) = -6.22;
  room_boundaries(17, 0) = -18.5;
  room_boundaries(17, 1) = -3.1;
  room_boundaries(18, 0) = -6.22;
  room_boundaries(18, 1) = -2.5;
  room_boundaries(19, 0) = -24;
  room_boundaries(19, 1) = -18.5;
  room_boundaries(20, 0) = -2.5;
  room_boundaries(20, 1) = -0;
  room_boundaries(21, 0) = -24;
  room_boundaries(21, 1) = 0;
};

}  // namespace matching_algorithms
}  // namespace cad_percept
