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
  cad_sub_ = nh_.subscribe(cad_topic, input_queue_size, &test_Matcher::gotCAD, this);
  // Get Publisher
  scan_pub_ = nh.advertise<sensor_msgs::PointCloud2>("corrected_scan", 2, true);
  map_pub_ = nh_.advertise<PointCloud>("map", 1, true);
}

void test_Matcher::gotCAD(const cgal_msgs::TriangleMeshStamped& cad_mesh_in) {
  // ROS_INFO_STREAM("CAD message received");
  std::cout << "Processing CAD mesh" << std::endl;
  std::string frame_id = cad_mesh_in.header.frame_id;
  cgal::msgToMeshModel(cad_mesh_in.mesh, &reference_mesh_);

  // Get transformation from /map to mesh
  try {
    tf_listener_.waitForTransform(tf_map_frame, frame_id, ros::Time(0), ros::Duration(5.0));
    tf_listener_.lookupTransform(tf_map_frame, frame_id, ros::Time(0),
                                 transform);  // get transformation at latest time T_map_to_frame
  } catch (tf::TransformException ex) {
    ROS_ERROR_STREAM("Couldn't find transformation to mesh system");
  }
  tf::matrixTFToEigen(transform.getBasis(), rotation);
  tf::vectorTFToEigen(transform.getOrigin(), translation);
  transformation = Eigen::Matrix4d::Identity();
  transformation.block(0, 0, 3, 3) = rotation;
  transformation.block(0, 3, 3, 1) = translation;
  cgal::eigenTransformationToCgalTransformation(
      transformation,
      &ctransformation);  // convert matrix4d to cgal transformation to use
  // member function
  reference_mesh_->transform(ctransformation);

  // Sample from mes
  PointCloud pointcloud;
  sampleFromReferenceFacets(map_sampling_density, &pointcloud);

  ref_dp = cpt_utils::pointCloudToDP(pointcloud);
  scan_pub_.publish(
      PointMatcher_ros::pointMatcherCloudToRosMsg<float>(ref_dp, tf_map_frame, ros::Time::now()));
}

void test_Matcher::sampleFromReferenceFacets(const int density, PointCloud* pointcloud) {
  std::cout << "Sampling started" << std::endl;
  // for(int i = 0; i<10; i++){
  //    pointcloud->push_back(pcl::PointXYZ(rand()%20, rand()%20, rand()%20));
  //}
  pointcloud->clear();  // make sure point cloud is empty

  // first consider empty reference set (whole point cloud)
  int n_points = reference_mesh_->getArea() * density;
  std::cout << "Density " << n_points << std::endl;
  cpt_utils::sample_pc_from_mesh(reference_mesh_->getMesh(), n_points, 0.0, pointcloud);
}
}  // namespace matching_algorithms
}  // namespace cad_percept