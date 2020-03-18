#include <cgal_conversions/eigen_conversions.h>
#include <cgal_conversions/mesh_conversions.h>
#include <cgal_definitions/mesh_model.h>
#include <cgal_msgs/TriangleMeshStamped.h>
#include <cpt_utils/pc_processing.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pointmatcher/PointMatcher.h>
#include <pointmatcher_ros/point_cloud.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <random>

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

cad_percept::cgal::MeshModel::Ptr reference_mesh_;
PointCloud sample_map;
PointCloud lidar_frame;

float density;
bool gotCAD = false;

std::string tf_map_frame;

void getCAD(const cgal_msgs::TriangleMeshStamped& cad_mesh_in);

int main(int argc, char** argv) {
  ros::init(argc, argv, "lidar_simulator");
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_("~");

  tf_map_frame = nh_private_.param<std::string>("tfMapFrame", "/map");
  std::cout << "///////////////////////////////////////////////" << std::endl;
  std::cout << "             LiDAR Simulator started         " << std::endl;
  std::cout << "///////////////////////////////////////////////" << std::endl;

  // Get mesh
  density = nh_private_.param<int>("mapSamplingDensity", 100);
  std::string cad_topic = nh_private_.param<std::string>("cadTopic", "fail");
  ros::Subscriber cad_sub_ = nh_.subscribe(cad_topic, 1, &getCAD);
  std::cout << "Wait for CAD" << std::endl;
  while (!gotCAD) {
    ros::spinOnce();
  }

  // Get ground_truth
  float x = nh_private_.param<float>("groundtruthx", 0);
  float y = nh_private_.param<float>("groundtruthy", 0);
  float z = nh_private_.param<float>("groundtruthz", 0);
  float roll = nh_private_.param<float>("groundtruthroll", 0);
  float yaw = nh_private_.param<float>("groundtruthyaw", 0);
  float pitch = nh_private_.param<float>("groundtruthpitch", 0);

  // Transform point cloud according to ground truth
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  Eigen::Vector3d translation(x, y, z);
  Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());

  Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

  transform.block(0, 0, 3, 3) = q.matrix();
  transform.block(0, 3, 3, 1) = translation;
  pcl::transformPointCloud(sample_map, lidar_frame, transform);
  std::cout << "Lidar frame transfomed according to ground truth data" << std::endl;

  // Add lidar properties
  float range_of_lidar = nh_private_.param<float>("range_of_lidar", 20);
  // Bin characteristic & visible
  bool usebins = nh_private_.param<bool>("usebins", false);
  if (usebins) {
    std::cout << "Simulate bins of LiDAR" << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud_in = lidar_frame;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud_in);
    int k = 1;
    std::vector<int> pointindex(k);
    std::vector<float> distance_to_point(k);

    int number_of_bin = nh_private_.param<float>("number_of_bin", 16);
    std::vector<float> bin_elevation = nh_private_.param<std::vector<float>>("bin_elevation", {0});
    float dtheta = nh_private_.param<float>("lidar_angualar_resolution", 1);
    float dr = 0.1;
    float x_unit;
    float y_unit;
    float z_unit;
    float PI_angle = (float)(M_PI / 180);
    pcl::PointXYZ intersection_point;
    lidar_frame.clear();
    for (int bin_num = 0; bin_num < number_of_bin; bin_num++) {
      for (float theta = 0; theta < 360; theta += dtheta) {
        x_unit = cos(bin_elevation[bin_num] * PI_angle) * cos(theta * PI_angle);
        y_unit = cos(bin_elevation[bin_num] * PI_angle) * sin(theta * PI_angle);
        z_unit = sin(bin_elevation[bin_num] * PI_angle);

        for (float r = 0.1; r < range_of_lidar; r += dr) {
          intersection_point.x = r * x_unit;
          intersection_point.y = r * y_unit;
          intersection_point.z = r * z_unit + 1.5;  // Add offset of LiDAR
          kdtree.nearestKSearch(intersection_point, k, pointindex, distance_to_point);

          if (distance_to_point[0] < 0.01) {
            lidar_frame.push_back(intersection_point);
            break;
          }
        }
      }
    }
  }

  // Sensor noise
  float noise_variance = nh_private_.param<float>("accuracy_of_lidar", 0.02);
  std::default_random_engine generator;
  std::normal_distribution<float> noise(0, noise_variance);

  std::cout << "Start to add lidar properties" << std::endl;
  PointCloud full_lidar_frame = lidar_frame;
  lidar_frame.clear();
  for (PointCloud::iterator i = full_lidar_frame.points.begin(); i < full_lidar_frame.points.end();
       i++) {
    if (sqrt(pow(i->x, 2) + pow(i->y, 2) + pow(i->z, 2)) < range_of_lidar) {
      i->x = i->x + noise(generator);
      i->y = i->y + noise(generator);
      i->z = i->z + noise(generator);

      lidar_frame.push_back(*i);
    }
  }

  // Publish simulated lidar frame & sampled map
  ros::Publisher scan_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("sim_rslidar_points", 1, true);
  ros::Publisher sample_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("sample_map", 1, true);

  DP ref_scan = cad_percept::cpt_utils::pointCloudToDP(lidar_frame);
  DP ref_sample_map = cad_percept::cpt_utils::pointCloudToDP(sample_map);

  scan_pub_.publish(
      PointMatcher_ros::pointMatcherCloudToRosMsg<float>(ref_scan, tf_map_frame, ros::Time::now()));
  sample_map_pub_.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(
      ref_sample_map, tf_map_frame, ros::Time::now()));

  std::cout << "LiDAR Simulator starts to publish" << std::endl;

  ros::spin();

  return 0;
}

// Get CAD and sample points
void getCAD(const cgal_msgs::TriangleMeshStamped& cad_mesh_in) {
  if (!gotCAD) {
    std::cout << "Processing CAD mesh" << std::endl;
    std::string frame_id = cad_mesh_in.header.frame_id;
    cad_percept::cgal::msgToMeshModel(cad_mesh_in.mesh, &reference_mesh_);

    // Get transformation from /map to mesh
    tf::StampedTransform transform;
    tf::TransformListener tf_listener_(ros::Duration(30));
    try {
      tf_listener_.waitForTransform(tf_map_frame, frame_id, ros::Time(0), ros::Duration(5.0));
      tf_listener_.lookupTransform(tf_map_frame, frame_id, ros::Time(0),
                                   transform);  // get transformation at latest time T_map_to_frame
    } catch (tf::TransformException ex) {
      ROS_ERROR_STREAM("Couldn't find transformation to mesh system");
    }

    // Transform CAD to map
    Eigen::Matrix3d rotation;
    tf::matrixTFToEigen(transform.getBasis(), rotation);
    Eigen::Vector3d translation;
    tf::vectorTFToEigen(transform.getOrigin(), translation);
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation.block(0, 0, 3, 3) = rotation;
    transformation.block(0, 3, 3, 1) = translation;
    cad_percept::cgal::Transformation ctransformation;
    cad_percept::cgal::eigenTransformationToCgalTransformation(
        transformation,
        &ctransformation);  // convert matrix4d to cgal transformation
    reference_mesh_->transform(ctransformation);

    // Sample from mesh
    sample_map.clear();
    int n_points = reference_mesh_->getArea() * density;
    cad_percept::cpt_utils::sample_pc_from_mesh(reference_mesh_->getMesh(), n_points, 0.0,
                                                &sample_map);

    std::cout << "CAD ready" << std::endl;
    gotCAD = true;
  }
}