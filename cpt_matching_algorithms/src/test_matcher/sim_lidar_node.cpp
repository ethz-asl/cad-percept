#include <cgal_conversions/eigen_conversions.h>
#include <cgal_conversions/mesh_conversions.h>
#include <cgal_conversions/tf_conversions.h>
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
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <random>

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

cad_percept::cgal::MeshModel::Ptr reference_mesh;
PointCloud lidar_scan;

bool got_CAD = false;
std::string tf_map_frame;

bool use_marker_for_sim;
bool fix_lidar_scan;

float x;
float y;
float z;
std::vector<float> quat;

float range_of_lidar;
bool use_bins;
std::vector<float> bin_elevation;
float dtheta;
float lidar_offset;
float noise_variance;

ros::Publisher scan_pub;

void getCAD(const cgal_msgs::TriangleMeshStamped &cad_mesh_in);
void sim_lidar_at_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
void simulate_lidar();

int main(int argc, char **argv) {
  ros::init(argc, argv, "lidar_simulator");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  tf_map_frame = nh_private.param<std::string>("tfMapFrame", "/map");
  std::cout << "///////////////////////////////////////////////" << std::endl;
  std::cout << "             LiDAR Simulator started         " << std::endl;
  std::cout << "///////////////////////////////////////////////" << std::endl;

  // Parameter from server
  range_of_lidar = nh_private.param<float>("range_of_lidar", 20);
  use_bins = nh_private.param<bool>("usebins", false);
  bin_elevation = nh_private.param<std::vector<float>>("bin_elevation", {0});
  dtheta = nh_private.param<float>("lidar_angular_resolution", 1);
  lidar_offset = nh_private.param<float>("lidar_offset", 1.5);
  noise_variance = nh_private.param<float>("accuracy_of_lidar", 0.02);

  use_marker_for_sim = nh_private.param<bool>("SimMarker", false);
  fix_lidar_scan = nh_private.param<bool>("FixLidarScans", false);
  ros::Subscriber marker_pos_sub;
  if (use_marker_for_sim) {
    std::cout << "Move marker to get lidar scans" << std::endl;
    std::string marker_pos_topic = nh_private.param<std::string>("SimMarkerPoseTopic", "fail");
    marker_pos_sub = nh.subscribe(marker_pos_topic, 20, &sim_lidar_at_marker);
    // Push back no rotation
    quat.push_back(1);
    quat.push_back(0);
    quat.push_back(0);
    quat.push_back(0);

  } else {
    x = nh_private.param<float>("groundtruthx", 0);
    y = nh_private.param<float>("groundtruthy", 0);
    z = nh_private.param<float>("groundtruthz", 0);
    quat = nh_private.param<std::vector<float>>("groundtruth_orientation", {});
  }

  // Get mesh
  std::string cad_topic = nh_private.param<std::string>("cadTopic", "fail");
  ros::Subscriber cad_sub = nh.subscribe(cad_topic, 1, &getCAD);
  std::cout << "Wait for CAD" << std::endl;

  scan_pub = nh.advertise<sensor_msgs::PointCloud2>("sim_rslidar_points", 1, true);

  ros::spin();

  return 0;
}

// Get CAD and sample points
void getCAD(const cgal_msgs::TriangleMeshStamped &cad_mesh_in) {
  if (!got_CAD) {
    std::cout << "Processing CAD mesh" << std::endl;
    std::string frame_id = cad_mesh_in.header.frame_id;
    cad_percept::cgal::msgToMeshModel(cad_mesh_in.mesh, &reference_mesh);

    // Get transformation from /map to mesh
    tf::StampedTransform transform;
    tf::TransformListener tf_listener(ros::Duration(30));
    try {
      tf_listener.waitForTransform(tf_map_frame, frame_id, ros::Time(0), ros::Duration(5.0));
      tf_listener.lookupTransform(tf_map_frame, frame_id, ros::Time(0),
                                  transform);  // get transformation at latest time T_map_to_frame
    } catch (tf::TransformException ex) {
      ROS_ERROR_STREAM("Couldn't find transformation to mesh system");
    }

    // Transform CAD to map
    cad_percept::cgal::Transformation ctransformation;
    cad_percept::cgal::tfTransformationToCGALTransformation(transform, ctransformation);
    reference_mesh->transform(ctransformation);

    std::cout << "CAD ready" << std::endl;
    got_CAD = true;

    if (!use_marker_for_sim) {
      simulate_lidar();
    }
  }
}

void sim_lidar_at_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  // Change position of lidar to the one of the marker (different coordinate system)
  x = -(feedback->pose.position.y) - 1;
  y = feedback->pose.position.x + 1;
  z = feedback->pose.position.z;

  std::cout << std::endl;
  std::cout << "Position of marker/lidar: " << x << " " << y << " " << z << std::endl;
  std::cout << std::endl;

  if (got_CAD) {
    simulate_lidar();
    lidar_scan.clear();
  }
}

void simulate_lidar() {
  cad_percept::cgal::Point origin;
  cad_percept::cgal::Transformation ctransformation;
  if (!fix_lidar_scan) {
    // Transform point cloud according to ground truth
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    Eigen::Vector3d translation(x, y, z);
    Eigen::Quaterniond q(quat[0], quat[1], quat[2], quat[3]);
    transform.block(0, 0, 3, 3) = q.matrix();
    transform.block(0, 3, 3, 1) = translation;
    cad_percept::cgal::eigenTransformationToCgalTransformation(transform, &ctransformation);
    reference_mesh->transform(ctransformation.inverse());

    if (use_marker_for_sim) {
      origin = cad_percept::cgal::Point(0, 0, 0);
    } else {
      origin = cad_percept::cgal::Point(0, 0, lidar_offset);
    }
  } else {
    if (use_marker_for_sim) {
      origin = cad_percept::cgal::Point(x, y, z);
    } else {
      origin = cad_percept::cgal::Point(x, y, z + lidar_offset);
    }
  }

  // Add lidar properties
  // Bin characteristic & visible
  if (use_bins) {
    std::cout << "Simulate bins of LiDAR" << std::endl;

    float x_unit;
    float y_unit;
    float z_unit;
    float PI_angle = (float)(M_PI / 180);
    cad_percept::cgal::Ray bin_ray;
    cad_percept::cgal::Point unit_dir;
    cad_percept::cgal::Intersection inter_point;
    pcl::PointXYZ pcl_inter_point;
    lidar_scan.clear();
    for (auto &bin : bin_elevation) {
      for (float theta = 0; theta < 360; theta += dtheta) {
        x_unit = cos(bin * PI_angle) * cos(theta * PI_angle) + origin.x();
        y_unit = cos(bin * PI_angle) * sin(theta * PI_angle) + origin.y();
        z_unit = sin(bin * PI_angle) + origin.z();
        unit_dir = cad_percept::cgal::Point(x_unit, y_unit, z_unit);
        bin_ray = cad_percept::cgal::Ray(origin, unit_dir);

        if (reference_mesh->isIntersection(bin_ray)) {
          inter_point = reference_mesh->getIntersection(bin_ray);
          pcl_inter_point.x = (float)inter_point.intersected_point.x();
          pcl_inter_point.y = (float)inter_point.intersected_point.y();
          pcl_inter_point.z = (float)inter_point.intersected_point.z();
          lidar_scan.push_back(pcl_inter_point);
        }
      }
    }
  }

  // Sensor noise
  std::default_random_engine generator;
  std::normal_distribution<float> noise(0, noise_variance);

  std::cout << "Start to add lidar properties" << std::endl;
  PointCloud full_lidar_scan = lidar_scan;
  lidar_scan.clear();
  for (auto i : full_lidar_scan.points) {
    if (sqrt(pow(i.x, 2) + pow(i.y, 2) + pow(i.z, 2)) < range_of_lidar) {
      i.x = i.x + noise(generator);
      i.y = i.y + noise(generator);
      i.z = i.z + noise(generator);

      lidar_scan.push_back(i);
    }
  }

  // Publish simulated lidar frame
  DP ref_scan = cad_percept::cpt_utils::pointCloudToDP(lidar_scan);
  scan_pub.publish(
      PointMatcher_ros::pointMatcherCloudToRosMsg<float>(ref_scan, tf_map_frame, ros::Time::now()));
  std::cout << "LiDAR Simulator starts to publish" << std::endl;

  if (!fix_lidar_scan) {
    // Transform mesh back for next iteration
    reference_mesh->transform(ctransformation);
  }
}