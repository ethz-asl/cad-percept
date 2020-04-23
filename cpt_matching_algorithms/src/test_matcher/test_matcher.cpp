#include "test_matcher/test_matcher.h"

namespace cad_percept {
namespace matching_algorithms {

TestMatcher::TestMatcher(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private) {
  /*//////////////////////////////////////
                 Setup
  ///////////////////////////////////////*/
  // Get Parameters from Server
  cad_topic_ = nh_private_.param<std::string>("cadTopic", "fail");
  lidar_topic_ = nh_private_.param<std::string>("lidarTopic", "fail");
  sim_lidar_topic_ = nh_private_.param<std::string>("simlidarTopic", "fail");
  ground_truth_topic_ = nh_private_.param<std::string>("groundtruthTopic", "fail");
  tf_map_frame_ = nh_private_.param<std::string>("tfMapFrame", "/map");
  tf_lidar_frame_ = nh_private_.param<std::string>("tfLidarFrame", "/marker_pose");
  use_sim_lidar_ = nh_private_.param<bool>("usetoyproblem", false);

  // Get Subscriber
  map_sub_ = nh_.subscribe(cad_topic_, 1, &TestMatcher::getCAD, this);
  lidar_sub_ = nh_.subscribe(lidar_topic_, 1, &TestMatcher::getLidar, this);
  lidar_sim_sub_ = nh_.subscribe(sim_lidar_topic_, 1, &TestMatcher::getSimLidar, this);
  gt_sub_ = nh_.subscribe(ground_truth_topic_, 1, &TestMatcher::getGroundTruth, this);

  // Get Publisher
  scan_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("matched_point_cloud", 1, true);
}

// Get CAD and sample points
void TestMatcher::getCAD(const cgal_msgs::TriangleMeshStamped &cad_mesh_in) {
  if (!map_ready_) {
    std::cout << "Processing CAD mesh" << std::endl;
    std::string frame_id = cad_mesh_in.header.frame_id;
    cad_percept::cgal::msgToMeshModel(cad_mesh_in.mesh, &reference_mesh_);

    // Get transformation from /map to mesh
    tf::StampedTransform transform;
    tf::TransformListener tf_listener(ros::Duration(30));
    try {
      tf_listener.waitForTransform(tf_map_frame_, frame_id, ros::Time(0), ros::Duration(5.0));
      tf_listener.lookupTransform(tf_map_frame_, frame_id, ros::Time(0),
                                  transform);  // get transformation at latest time T_map_to_frame
    } catch (tf::TransformException ex) {
      ROS_ERROR_STREAM("Couldn't find transformation to mesh system");
    }

    cad_percept::cgal::Transformation ctransformation;
    cgal::tfTransformationToCGALTransformation(transform, ctransformation);
    reference_mesh_->transform(ctransformation);

    // Sample from mesh
    sample_map_.clear();
    sample_density_ = nh_private_.param<int>("mapSamplingDensity", 20);
    int n_points = reference_mesh_->getArea() * sample_density_;
    cad_percept::cpt_utils::sample_pc_from_mesh(reference_mesh_->getMesh(), n_points, 0.0,
                                                &sample_map_);

    std::string file_name = nh_private_.param<std::string>("map_plane_file", "fail");

    // load data from file
    map_planes_ = new MapPlanes();
    map_planes_->loadFromYamlFile(file_name);
    map_planes_->dispAllPlanes();

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
void TestMatcher::getLidar(const sensor_msgs::PointCloud2 &lidar_scan_p2) {
  if (!lidar_scan_ready_ && !use_sim_lidar_) {
    std::cout << "Processing lidar frame" << std::endl;

    // Convert PointCloud2 to PointCloud
    pcl::PCLPointCloud2 lidar_pc2;
    pcl_conversions::toPCL(lidar_scan_p2, lidar_pc2);
    pcl::fromPCLPointCloud2(lidar_pc2, lidar_scan_);
    std::vector<int> nan_indices;
    pcl::removeNaNFromPointCloud(lidar_scan_, lidar_scan_, nan_indices);
    if (nan_indices.size() != 0) {
      std::cout << "Attention: Detected NaNs in the given point cloud. Removed "
                   "this values..."
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
void TestMatcher::getGroundTruth(const geometry_msgs::PointStamped &gt_in) {
  if (!ground_truth_ready_ && !use_sim_lidar_) {
    // Transform gt into map frame
    ground_truth_[0] = -gt_in.point.x;
    ground_truth_[1] = -gt_in.point.y;
    ground_truth_[2] = gt_in.point.z;
    std::cout << "Got ground truth data" << std::endl;
    ground_truth_ready_ = true;

    if (ready_for_eval_) {
      evaluate();
    }
  }
}

// Get LiDAR data and ground truth from simulator
void TestMatcher::getSimLidar(const sensor_msgs::PointCloud2 &lidar_scan_p2) {
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

    // Ground truth is T_map,lidar
    tf::quaternionTFToEigen(transform.getRotation(), gt_quat_);
    tf::vectorTFToEigen(transform.getOrigin(), ground_truth_);

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
    std::vector<Eigen::Vector3d> plane_normals;
    std::string extractor = nh_private_.param<std::string>("PlaneExtractor", "fail");
    if (!extractor.compare("pclPlaneExtraction")) {
      PlaneExtractor::pclPlaneExtraction(extracted_planes, plane_normals, lidar_scan_,
                                         tf_lidar_frame_, plane_pub_);
    } else if (!extractor.compare("rhtPlaneExtraction")) {
      PlaneExtractor::rhtPlaneExtraction(extracted_planes, plane_normals, lidar_scan_,
                                         tf_lidar_frame_, plane_pub_);
    } else if (!extractor.compare("iterRhtPlaneExtraction")) {
      PlaneExtractor::iterRhtPlaneExtraction(extracted_planes, plane_normals, lidar_scan_,
                                             tf_lidar_frame_, plane_pub_);

    } else if (!extractor.compare("cgalRegionGrowing")) {
      PlaneExtractor::cgalRegionGrowing(extracted_planes, plane_normals, lidar_scan_,
                                        tf_lidar_frame_, plane_pub_);
    } else {
      std::cout << "Error: Could not find given plane extractor" << std::endl;
      return;
    }

    // Convert planes to PointNormal PointCloud
    pcl::PointNormal norm_point;
    pcl::PointXYZ plane_centroid;
    int plane_nr = 0;
    for (auto plane_normal : plane_normals) {
      pcl::computeCentroid(extracted_planes[plane_nr], plane_centroid);
      norm_point.x = plane_centroid.x;
      norm_point.y = plane_centroid.y;
      norm_point.z = plane_centroid.z;

      norm_point.normal_x = plane_normal[0];
      norm_point.normal_y = plane_normal[1];
      norm_point.normal_z = plane_normal[2];

      // Correct normals considering normal direction convention for matching
      if (norm_point.x * norm_point.normal_x + norm_point.y * norm_point.normal_y +
              norm_point.z * norm_point.normal_z <
          0) {
        norm_point.normal_x = -norm_point.normal_x;
        norm_point.normal_y = -norm_point.normal_y;
        norm_point.normal_z = -norm_point.normal_z;
      }

      scan_planes_.push_back(norm_point);
      plane_nr++;
    }

    std::string plane_matcher = nh_private_.param<std::string>("PlaneMatch", "fail");
    std::cout << plane_matcher << std::endl;
    pcl::PointCloud<pcl::PointNormal> map_planes;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> room_boundaries;
    map_planes_->getMapPlaneInformations(map_planes, room_boundaries);
    // Plane Matching (Get T_map,lidar)
    if (!plane_matcher.compare("IntersectionPatternMatcher")) {
      PlaneMatch::IntersectionPatternMatcher(transform_TR_, scan_planes_, map_planes,
                                             room_boundaries);
    } else if (!plane_matcher.compare("useMatchSolution")) {
      PlaneMatch::loadExampleSol(transform_TR_, scan_planes_, map_planes);
    } else if (!plane_matcher.compare("LineSegmentRansac")) {
      PlaneMatch::LineSegmentRansac(transform_TR_, scan_planes_, map_planes, room_boundaries);
    } else {
      std::cout << "Error: Could not find given plane matcher" << std::endl;
      return;
    }
  } else {
    std::cout << "Error: Could not find given matcher" << std::endl;
    return;
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

  pcl::transformPointCloud(lidar_scan_, lidar_scan_, res_transform);

  /*//////////////////////////////////////
                Visualization
  ///////////////////////////////////////*/

  DP ref_dp = cpt_utils::pointCloudToDP(lidar_scan_);
  scan_pub_.publish(
      PointMatcher_ros::pointMatcherCloudToRosMsg<float>(ref_dp, tf_map_frame_, ros::Time::now()));
  ready_for_eval_ = true;
}

void TestMatcher::evaluate() {
  /*//////////////////////////////////////
                Evaluation
  ///////////////////////////////////////*/

  std::cout << "calculated position: x: " << transform_TR_[0] << " y: " << transform_TR_[1]
            << " z: " << transform_TR_[2] << std::endl;
  std::cout << "ground truth position: x: " << ground_truth_[0] << " y: " << ground_truth_[1]
            << " z: " << ground_truth_[2] << std::endl;
  std::cout << "calculated orientation: qw: " << transform_TR_[3] << " qx: " << transform_TR_[4]
            << " qy: " << transform_TR_[5] << " qz: " << transform_TR_[6] << std::endl;

  if (use_sim_lidar_) {
    std::cout << "ground truth orientation: qw: " << gt_quat_.w() << " qx: " << gt_quat_.x()
              << " qy: " << gt_quat_.y() << " qz: " << gt_quat_.z() << std::endl;
  }

  float error = sqrt(pow(transform_TR_[0] - ground_truth_[0], 2) +
                     pow(transform_TR_[1] - ground_truth_[1], 2) +
                     pow(transform_TR_[2] - ground_truth_[2], 2));
  getError(sample_map_, lidar_scan_);
  std::cout << "error (euclidean distance of translation): " << error << std::endl;
}

// Get RMSE of two point clouds
void TestMatcher::getError(PointCloud p1, PointCloud p2) {
  DP p1_dp = cpt_utils::pointCloudToDP(p1);
  DP p2_dp = cpt_utils::pointCloudToDP(p2);

  // Calculation of Hausdorff Distance withot oulier removal, source:
  // cpt_selective_icp::mapper
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

}  // namespace matching_algorithms
}  // namespace cad_percept
