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
PointCloud lidar_frame;

bool got_CAD = false;
std::string tf_map_frame;

float x;
float y;
float z;
float roll;
float yaw;
float pitch;

float range_of_lidar;
bool use_bins;
int number_of_bin;
std::vector<float> bin_elevation;
float dtheta;
float lidar_offset;
float noise_variance;

ros::Publisher scan_pub_;

void getCAD(const cgal_msgs::TriangleMeshStamped &cad_mesh_in);
void simulate_lidar();

int main(int argc, char **argv) {
  ros::init(argc, argv, "lidar_simulator");
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_("~");

  tf_map_frame = nh_private_.param<std::string>("tfMapFrame", "/map");
  std::cout << "///////////////////////////////////////////////" << std::endl;
  std::cout << "             LiDAR Simulator started         " << std::endl;
  std::cout << "///////////////////////////////////////////////" << std::endl;

  // Parameter from server
  x = nh_private_.param<float>("groundtruthx", 0);
  y = nh_private_.param<float>("groundtruthy", 0);
  z = nh_private_.param<float>("groundtruthz", 0);
  roll = nh_private_.param<float>("groundtruthroll", 0);
  yaw = nh_private_.param<float>("groundtruthyaw", 0);
  pitch = nh_private_.param<float>("groundtruthpitch", 0);

  range_of_lidar = nh_private_.param<float>("range_of_lidar", 20);
  use_bins = nh_private_.param<bool>("usebins", false);
  number_of_bin = nh_private_.param<float>("number_of_bin", 16);
  bin_elevation = nh_private_.param<std::vector<float>>("bin_elevation", {0});
  dtheta = nh_private_.param<float>("lidar_angualar_resolution", 1);
  lidar_offset = nh_private_.param<float>("lidar_offset", 1.5);
  noise_variance = nh_private_.param<float>("accuracy_of_lidar", 0.02);

  scan_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("sim_rslidar_points", 1, true);

  // Get mesh
  std::string cad_topic = nh_private_.param<std::string>("cadTopic", "fail");
  ros::Subscriber cad_sub_ = nh_.subscribe(cad_topic, 1, &getCAD);
  std::cout << "Wait for CAD" << std::endl;

  ros::spin();

  return 0;
}

// Get CAD and sample points
void getCAD(const cgal_msgs::TriangleMeshStamped &cad_mesh_in) {
  if (!got_CAD) {
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

    std::cout << "CAD ready" << std::endl;
    got_CAD = true;

    simulate_lidar();
  }
}

void simulate_lidar() {
  // Transform point cloud according to ground truth
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  Eigen::Vector3d translation(x, y, z);
  Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());

  Eigen::Quaternion<double> q = roll_angle * yaw_angle * pitch_angle;

  transform.block(0, 0, 3, 3) = q.matrix();
  transform.block(0, 3, 3, 1) = translation;
  cad_percept::cgal::Transformation ctransformation;
  cad_percept::cgal::eigenTransformationToCgalTransformation(transform, &ctransformation);
  reference_mesh_->transform(ctransformation);

  std::cout << "Lidar frame transfomed according to ground truth data" << std::endl;

  // Add lidar properties
  // Bin characteristic & visible
  if (use_bins) {
    std::cout << "Simulate bins of LiDAR" << std::endl;

    float x_unit;
    float y_unit;
    float z_unit;
    float PI_angle = (float)(M_PI / 180);
    cad_percept::cgal::Ray bin_ray;
    cad_percept::cgal::Point origin(0, 0, lidar_offset);
    cad_percept::cgal::Point unit_dir;
    cad_percept::cgal::Intersection inter_point;
    pcl::PointXYZ pcl_inter_point;
    lidar_frame.clear();
    for (int bin_num = 0; bin_num < number_of_bin; bin_num++) {
      for (float theta = 0; theta < 360; theta += dtheta) {
        x_unit = cos(bin_elevation[bin_num] * PI_angle) * cos(theta * PI_angle);
        y_unit = cos(bin_elevation[bin_num] * PI_angle) * sin(theta * PI_angle);
        z_unit = sin(bin_elevation[bin_num] * PI_angle) + lidar_offset;
        unit_dir = cad_percept::cgal::Point(x_unit, y_unit, z_unit);
        bin_ray = cad_percept::cgal::Ray(origin, unit_dir);

        if (reference_mesh_->isIntersection(bin_ray)) {
          inter_point = reference_mesh_->getIntersection(bin_ray);
          pcl_inter_point.x = (float)inter_point.intersected_point.x();
          pcl_inter_point.y = (float)inter_point.intersected_point.y();
          pcl_inter_point.z = (float)inter_point.intersected_point.z();
          lidar_frame.push_back(pcl_inter_point);
        }
      }
    }
  }

  // Sensor noise
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

  // Publish simulated lidar frame
  DP ref_scan = cad_percept::cpt_utils::pointCloudToDP(lidar_frame);
  scan_pub_.publish(
      PointMatcher_ros::pointMatcherCloudToRosMsg<float>(ref_scan, tf_map_frame, ros::Time::now()));

  std::cout << "LiDAR Simulator starts to publish" << std::endl;
}