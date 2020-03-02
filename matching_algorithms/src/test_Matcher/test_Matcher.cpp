#include "test_Matcher/test_Matcher.h"

namespace cad_percept {
namespace matching_algorithms {

test_Matcher::test_Matcher(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), tf_listener_(ros::Duration(30)) {
  // Get Parameters from Server
  cad_topic = nh_private_.param<std::string>("cadTopic", "fail");
  input_queue_size = nh_private_.param<int>("inputQueueSize", 10);
  map_sampling_density = nh_private_.param<int>("mapSamplingDensity", 100);
  tf_map_frame = nh_private_.param<std::string>("tfMapFrame", "/map");

  // Get Subscriber
  cad_sub_ = nh_.subscribe(cad_topic, input_queue_size, &test_Matcher::getCAD, this);
  lidar_sub_ = nh.subscribe("rslidar_points", input_queue_size, &test_Matcher::getLiDAR, this);
  // Get Publisher
  scan_pub_ = nh.advertise<sensor_msgs::PointCloud2>("corrected_scan", 2, true);
  map_pub_ = nh_.advertise<PointCloud>("map", 1, true);

  std::cout << "Wait for Map and Lidar frame" << std::endl;
  while (!CAD_ready || !lidar_frame_ready) {
    ros::spinOnce();
  };
  std::cout << "Received LiDAR and CAD: Start to process data" << std::endl;
  float transformTR[6] = {0, 0, 0, 0, 0, 0};  // x y z roll pitch yaw
  matcher(transformTR);

  for (int i = 0; i < 6; i++) {
    std::cout << transformTR[i];
  }
  std::cout << std::endl;

  // Transform LiDAR frame
  Eigen::Affine3f affinetransform =
      pcl::getTransformation(transformTR[0], transformTR[1], transformTR[2], transformTR[3],
                             transformTR[4], transformTR[5]);

  pcl::transformPointCloud(lidar_frame, lidar_frame, affinetransform);

  ref_dp = cpt_utils::pointCloudToDP(lidar_frame);
  scan_pub_.publish(
      PointMatcher_ros::pointMatcherCloudToRosMsg<float>(ref_dp, tf_map_frame, ros::Time::now()));

  // Check with ground truth position and give out error
  std::cout << "error: " << std::endl;

}  // namespace matching_algorithms

void test_Matcher::matcher(float (&transformTR)[6]) { transformTR[2] = 11.0; }

// Preprocessing
void test_Matcher::getCAD(const cgal_msgs::TriangleMeshStamped& cad_mesh_in) {
  if (!gotCAD) {
    std::cout << "Processing CAD mesh" << std::endl;
    std::string frame_id = cad_mesh_in.header.frame_id;
    cgal::msgToMeshModel(cad_mesh_in.mesh, &reference_mesh_);

    // Get transformation from /map to mesh
    tf::StampedTransform transform;
    try {
      tf_listener_.waitForTransform(tf_map_frame, frame_id, ros::Time(0), ros::Duration(5.0));
      tf_listener_.lookupTransform(tf_map_frame, frame_id, ros::Time(0),
                                   transform);  // get transformation at latest time T_map_to_frame
    } catch (tf::TransformException ex) {
      ROS_ERROR_STREAM("Couldn't find transformation to mesh system");
    }
    Eigen::Matrix3d rotation;
    tf::matrixTFToEigen(transform.getBasis(), rotation);
    Eigen::Vector3d translation;
    tf::vectorTFToEigen(transform.getOrigin(), translation);
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation.block(0, 0, 3, 3) = rotation;
    transformation.block(0, 3, 3, 1) = translation;
    cgal::Transformation ctransformation;
    cgal::eigenTransformationToCgalTransformation(
        transformation,
        &ctransformation);  // convert matrix4d to cgal transformation
    reference_mesh_->transform(ctransformation);

    // Sample from mesh
    sampleFromReferenceFacets(map_sampling_density, &sample_map);

    std::cout << "CAD ready" << std::endl;
    CAD_ready = true;
  }
}

void test_Matcher::getLiDAR(const sensor_msgs::PointCloud2& lidarframe) {
  if (!gotlidar) {
    gotlidar = true;
    std::cout << "Processing lidar frame" << std::endl;

    // Convert PointCloud2 to PointCloud
    pcl::PCLPointCloud2 lidar_pc2;
    pcl_conversions::toPCL(lidarframe, lidar_pc2);
    pcl::fromPCLPointCloud2(lidar_pc2, lidar_frame);

    // Correct for scale difference in lidar frame and map
    float scale_lidar_map = 1.5;  // Will ask for correct scale in the next meeting
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform(0, 0) = transform(0, 0) * scale_lidar_map;
    transform(1, 1) = transform(1, 1) * scale_lidar_map;
    transform(2, 2) = transform(2, 2) * scale_lidar_map;
    pcl::transformPointCloud(lidar_frame, lidar_frame, transform);

    std::cout << "Lidar frame ready" << std::endl;
    lidar_frame_ready = true;
  }
}

void test_Matcher::sampleFromReferenceFacets(const int density, PointCloud* pointcloud) {
  pointcloud->clear();  // make sure point cloud is empty
  int n_points = reference_mesh_->getArea() * density;
  cpt_utils::sample_pc_from_mesh(reference_mesh_->getMesh(), n_points, 0.0, pointcloud);
}
}  // namespace matching_algorithms
}  // namespace cad_percept