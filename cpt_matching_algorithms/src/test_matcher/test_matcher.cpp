#include "test_matcher/test_matcher.h"

namespace cad_percept {
namespace matching_algorithms {

TestMatcher::TestMatcher(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
  /*//////////////////////////////////////
                 Setup
  ///////////////////////////////////////*/
  // Get Parameters from Server
  cad_topic = nh_private_.param<std::string>("cadTopic", "fail");
  input_queue_size = nh_private_.param<int>("inputQueueSize", 10);
  tf_map_frame = nh_private_.param<std::string>("tfMapFrame", "/map");
  use_sim_lidar = nh_private_.param<bool>("usetoyproblem", false);

  // Get Subscriber
  sample_map_sub_ = nh_.subscribe("sample_map", input_queue_size, &TestMatcher::getsampleCAD, this);
  lidar_sub_ = nh_.subscribe("rslidar_points", input_queue_size, &TestMatcher::getLiDAR, this);
  lidar_sim_sub_ =
      nh_.subscribe("sim_rslidar_points", input_queue_size, &TestMatcher::getsimLiDAR, this);
  gt_sub_ = nh_.subscribe("ground_truth", input_queue_size, &TestMatcher::getGroundTruth, this);

  // Get Publisher
  scan_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("matched_point_cloud", input_queue_size, true);

  // Get Map and Lidar frame
  std::cout << "Wait for Map and Lidar frame" << std::endl;
  while (!map_ready || !lidar_frame_ready) {
    ros::spinOnce();
  };
  std::cout << "Received LiDAR and CAD as point cloud" << std::endl;

  /*//////////////////////////////////////
                 Matching
  ///////////////////////////////////////*/

  // Find position of robot (change executable in cmake to change matcher)
  float transform_TR[6] = {0, 0, 0, 0, 0, 0};  // x y z roll pitch yaw

  // Selection of mapper
  if (nh_private_.param<bool>("use_template", false)) {
    template_match(transform_TR);
  }

  /*//////////////////////////////////////
                Transformation
  ///////////////////////////////////////*/

  // Transform LiDAR frame
  Eigen::Affine3f affine_transform =
      pcl::getTransformation(transform_TR[0], transform_TR[1], transform_TR[2], transform_TR[3],
                             transform_TR[4], transform_TR[5]);
  pcl::transformPointCloud(lidar_frame, lidar_frame, affine_transform.inverse());

  /*//////////////////////////////////////
                Visualization
  ///////////////////////////////////////*/
  ref_dp = cpt_utils::pointCloudToDP(lidar_frame);
  scan_pub_.publish(
      PointMatcher_ros::pointMatcherCloudToRosMsg<float>(ref_dp, tf_map_frame, ros::Time::now()));

  // Check with ground truth position and give out error
  std::cout << "Waiting for ground truth data" << std::endl;
  while (!ground_truth_ready) {
    ros::spinOnce();
  }

  /*//////////////////////////////////////
                Evaluation
  ///////////////////////////////////////*/

  std::cout << "calculated position: x: " << transform_TR[0] << " y: " << transform_TR[1]
            << " z: " << transform_TR[2] << std::endl;
  if (use_sim_lidar)
    std::cout << "calculated pose: roll: " << transform_TR[3] << " pitch: " << transform_TR[4]
              << " yaw: " << transform_TR[5] << std::endl;

  std::cout << "ground truth position: x: " << ground_truth.point.x
            << " y: " << ground_truth.point.y << " z: " << ground_truth.point.z << std::endl;
  if (use_sim_lidar)
    std::cout << "ground truth pose: roll: " << gt_roll << " pitch: " << gt_pitch
              << " yaw: " << gt_yaw << std::endl;

  float error = sqrt(pow(transform_TR[0] - ground_truth.point.x, 2) +
                     pow(transform_TR[1] - ground_truth.point.y, 2) +
                     pow(transform_TR[2] - ground_truth.point.z, 2));
  getError(sample_map, lidar_frame);
  std::cout << "error (euclidean distance of translation): " << error << std::endl;
}

// Get sampled mesh
void TestMatcher::getsampleCAD(const sensor_msgs::PointCloud2& cad_map) {
  if (!map_ready) {
    std::cout << "Processing map point cloud" << std::endl;

    // Convert PointCloud2 to PointCloud
    pcl::PCLPointCloud2 map_pc2;
    pcl_conversions::toPCL(cad_map, map_pc2);
    pcl::fromPCLPointCloud2(map_pc2, sample_map);

    std::cout << "Lidar frame ready" << std::endl;
    map_ready = true;
  }
}

// Get real LiDAR data
void TestMatcher::getLiDAR(const sensor_msgs::PointCloud2& lidar_frame_p2) {
  if (!lidar_frame_ready && !use_sim_lidar) {
    std::cout << "Processing lidar frame" << std::endl;

    // Convert PointCloud2 to PointCloud
    pcl::PCLPointCloud2 lidar_pc2;
    pcl_conversions::toPCL(lidar_frame_p2, lidar_pc2);
    pcl::fromPCLPointCloud2(lidar_pc2, lidar_frame);

    std::cout << "Lidar frame ready" << std::endl;
    lidar_frame_ready = true;
  }
}

// Get real ground truth position
void TestMatcher::getGroundTruth(const geometry_msgs::PointStamped& gt_in) {
  if (!ground_truth_ready && !use_sim_lidar) {
    ground_truth = gt_in;
    std::cout << "Got ground truth data" << std::endl;
    ground_truth_ready = true;
  }
}

// Get LiDAR data and ground truth from simulator
void TestMatcher::getsimLiDAR(const sensor_msgs::PointCloud2& lidar_frame_p2) {
  if (!lidar_frame_ready && use_sim_lidar) {
    std::cout << "Processing lidar frame" << std::endl;

    // Convert PointCloud2 to PointCloud
    pcl::PCLPointCloud2 lidar_pc2;
    pcl_conversions::toPCL(lidar_frame_p2, lidar_pc2);
    pcl::fromPCLPointCloud2(lidar_pc2, lidar_frame);

    std::cout << "Lidar frame ready" << std::endl;

    ground_truth.point.x = nh_private_.param<float>("groundtruthx", 0);
    ground_truth.point.y = nh_private_.param<float>("groundtruthy", 0);
    ground_truth.point.z = nh_private_.param<float>("groundtruthz", 0);
    gt_roll = nh_private_.param<float>("groundtruthroll", 0);
    gt_yaw = nh_private_.param<float>("groundtruthyaw", 0);
    gt_pitch = nh_private_.param<float>("groundtruthpitch", 0);

    std::cout << "Got ground truth data" << std::endl;

    lidar_frame_ready = true;
    ground_truth_ready = true;
  }
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