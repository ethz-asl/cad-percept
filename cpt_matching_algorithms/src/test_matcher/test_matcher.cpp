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
  sample_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("sample_map", 1, true);
  plane_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("extracted_planes", 1, true);
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

    // Transform CAD to map
    cad_percept::cgal::Transformation ctransformation;
    cgal::tfTransformationToCGALTransformation(transform, ctransformation);
    reference_mesh_->transform(ctransformation);

    // Sample from mesh
    sample_map_.clear();
    sample_density_ = nh_private_.param<int>("mapSamplingDensity", 100);
    int n_points = reference_mesh_->getArea() * sample_density_;
    cad_percept::cpt_utils::sample_pc_from_mesh(reference_mesh_->getMesh(), n_points, 0.0,
                                                &sample_map_);

    // Get planes from mesh
    // Extract all coplanar facets
    std::unordered_map<std::string, std::string> facetToPlane;
    std::unordered_multimap<std::string, std::string> planeToFacets;
    reference_mesh_->findAllCoplanarFacets(&facetToPlane, &planeToFacets, 0.1);
    std::cout << "Extracted coplanar facets" << std::endl;

    // // Extract planes
    // bool found_at_least_one_facet = true;
    // cgal::Plane actual_plane;
    // Eigen::Vector3d point_on_plane;
    // Eigen::Vector3d normal_of_plane;
    // pcl::PointNormal norm_point;

    // for (int plane_nr = 0; found_at_least_one_facet; ++plane_nr) {
    //   found_at_least_one_facet = false;
    //   for (auto itr = planeToFacets.begin(); itr != planeToFacets.end(); ++itr) {
    //     // Search for at least one facet of plane
    //     if (!(itr->first.compare(std::to_string(plane_nr)))) {
    //       actual_plane = reference_mesh_->getPlane(itr->second);
    //       point_on_plane = cgal::cgalPointToEigenVector(actual_plane.point());
    //       normal_of_plane =
    //           cgal::cgalVectorToEigenVector(actual_plane.orthogonal_vector()).normalized();

    //       norm_point.x = point_on_plane(0);
    //       norm_point.y = point_on_plane(1);
    //       norm_point.z = point_on_plane(2);
    //       norm_point.normal_x = normal_of_plane(0);
    //       norm_point.normal_y = normal_of_plane(1);
    //       norm_point.normal_z = normal_of_plane(2);

    //       map_planes_.push_back(norm_point);
    //       std::cout << "New plane added from mesh" << std::endl;
    //       found_at_least_one_facet = true;
    //       break;
    //     }
    //   }
    // }

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
    std::cout << "Processing lidar scan" << std::endl;

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
      // lidar_scan_p2.fields[3].name = "intensity";
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
  if (nh_private_.param<bool>("usetemplate", false)) {
    template_match();
  }
  if (nh_private_.param<bool>("useGoICP", false)) {
    go_icp_match();
  }
  //  if (nh_private_.param<bool>("useSuper4PCS", false)) {
  //    super4pcs_match();
  //  }
  if (nh_private_.param<bool>("usePlaneMatcher", false)) {
    // Filtering / Preprocessing Point Cloud
    if (nh_private_.param<bool>("useStructureFilter", false)) {
      int structure_threshold = nh_private_.param<int>("StructureThreshold", 150);
      CloudFilterLib::static_object_filter(structure_threshold, lidar_scan_,
                                           static_structure_cloud_);
    }
    if (nh_private_.param<bool>("useVoxelCentroidFilter", false)) {
      float search_radius = nh_private_.param<float>("Voxelsearchradius", 0.01);
      CloudFilterLib::voxel_centroid_filter(search_radius, lidar_scan_);
    }

    // Plane Extraction
    std::vector<pcl::PointCloud<pcl::PointXYZ>> extracted_planes;
    std::vector<std::vector<double>> plane_coefficients;
    if (nh_private_.param<bool>("usepclPlaneExtraction", false)) {
      PlaneExtractionLib::pcl_plane_extraction(extracted_planes, plane_coefficients, lidar_scan_,
                                               plane_pub_, tf_map_frame_, nh_private_);
    }
    if (nh_private_.param<bool>("useRHTPlaneExtraction", false)) {
      PlaneExtractionLib::rht_plane_extraction(extracted_planes, plane_coefficients, lidar_scan_,
                                               plane_pub_, tf_map_frame_, nh_private_);
    }
    if (nh_private_.param<bool>("useiterRHTPlaneExtraction", false)) {
      PlaneExtractionLib::iter_rht_plane_extraction(extracted_planes, plane_coefficients,
                                                    lidar_scan_, plane_pub_, tf_map_frame_,
                                                    nh_private_);
    }
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
}

void TestMatcher::evaluate() {
  /*//////////////////////////////////////
                Evaluation
  ///////////////////////////////////////*/

  std::cout << "calculated position: x: " << transform_TR_[0] << " y: " << transform_TR_[1]
            << " z: " << transform_TR_[2] << std::endl;

  std::cout << "ground truth position: x: " << ground_truth_.point.x
            << " y: " << ground_truth_.point.y << " z: " << ground_truth_.point.z << std::endl;

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
}  // namespace matching_algorithms
}  // namespace cad_percept