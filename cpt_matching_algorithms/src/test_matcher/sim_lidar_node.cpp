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
#include <random>

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

cad_percept::cgal::MeshModel::Ptr reference_mesh;
PointCloud lidar_frame;

bool got_CAD = false;

std::string tf_map_frame;

void getCAD(const cgal_msgs::TriangleMeshStamped& cad_mesh_in);

int main(int argc, char** argv) {
  ros::init(argc, argv, "lidar_simulator");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  tf_map_frame = nh_private.param<std::string>("tfMapFrame", "/map");
  std::cout << "///////////////////////////////////////////////" << std::endl;
  std::cout << "             LiDAR Simulator started         " << std::endl;
  std::cout << "///////////////////////////////////////////////" << std::endl;

  // Get mesh
  std::string cad_topic = nh_private.param<std::string>("cadTopic", "fail");
  ros::Subscriber cad_sub = nh.subscribe(cad_topic, 1, &getCAD);
  std::cout << "Wait for CAD" << std::endl;
  while (!got_CAD) {
    ros::spinOnce();
  }

  // Get ground_truth
  float x = nh_private.param<float>("groundtruthx", 0);
  float y = nh_private.param<float>("groundtruthy", 0);
  float z = nh_private.param<float>("groundtruthz", 0);
  std::vector<float> quat = nh_private.param<std::vector<float>>("groundtruth_orientation", {});

  // Transform point cloud according to ground truth
  cad_percept::cgal::Transformation ctransformation;
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  Eigen::Vector3d translation(x, y, z);
  Eigen::Quaterniond q(quat[0], quat[1], quat[2], quat[3]);
  transform.block(0, 0, 3, 3) = q.matrix();
  transform.block(0, 3, 3, 1) = translation;
  cad_percept::cgal::eigenTransformationToCgalTransformation(transform, &ctransformation);
  reference_mesh->transform(ctransformation);

  std::cout << "Lidar frame transfomed according to ground truth data" << std::endl;

  // Add lidar properties
  float range_of_lidar = nh_private.param<float>("range_of_lidar", 20);
  // Bin characteristic & visible
  bool use_bins = nh_private.param<bool>("usebins", false);
  if (use_bins) {
    std::cout << "Simulate bins of LiDAR" << std::endl;

    std::vector<float> bin_elevation = nh_private.param<std::vector<float>>("bin_elevation", {0});
    float dtheta = nh_private.param<float>("lidar_angular_resolution", 1);
    float lidar_offset = nh_private.param<float>("lidar_offset", 1.5);
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
    for (auto& bin : bin_elevation) {
      for (float theta = 0; theta < 360; theta += dtheta) {
        x_unit = cos(bin * PI_angle) * cos(theta * PI_angle);
        y_unit = cos(bin * PI_angle) * sin(theta * PI_angle);
        z_unit = sin(bin * PI_angle) + lidar_offset;
        unit_dir = cad_percept::cgal::Point(x_unit, y_unit, z_unit);
        bin_ray = cad_percept::cgal::Ray(origin, unit_dir);

        if (reference_mesh->isIntersection(bin_ray)) {
          inter_point = reference_mesh->getIntersection(bin_ray);
          pcl_inter_point.x = (float)inter_point.intersected_point.x();
          pcl_inter_point.y = (float)inter_point.intersected_point.y();
          pcl_inter_point.z = (float)inter_point.intersected_point.z();
          lidar_frame.push_back(pcl_inter_point);
        }
      }
    }
  }

  // Sensor noise
  float noise_variance = nh_private.param<float>("accuracy_of_lidar", 0.02);
  std::default_random_engine generator;
  std::normal_distribution<float> noise(0, noise_variance);

  std::cout << "Start to add lidar properties" << std::endl;
  PointCloud full_lidar_frame = lidar_frame;
  lidar_frame.clear();
  for (auto i : full_lidar_frame.points) {
    if (sqrt(pow(i.x, 2) + pow(i.y, 2) + pow(i.z, 2)) < range_of_lidar) {
      i.x = i.x + noise(generator);
      i.y = i.y + noise(generator);
      i.z = i.z + noise(generator);

      lidar_frame.push_back(i);
    }
  }

  // Publish simulated lidar frame
  ros::Publisher scan_pub = nh.advertise<sensor_msgs::PointCloud2>("sim_rslidar_points", 1, true);
  DP ref_scan = cad_percept::cpt_utils::pointCloudToDP(lidar_frame);
  scan_pub.publish(
      PointMatcher_ros::pointMatcherCloudToRosMsg<float>(ref_scan, tf_map_frame, ros::Time::now()));

  std::cout << "LiDAR Simulator starts to publish" << std::endl;

  ros::spin();

  return 0;
}

// Get CAD and sample points
void getCAD(const cgal_msgs::TriangleMeshStamped& cad_mesh_in) {
  if (!got_CAD) {
    std::cout << "Processing CAD mesh" << std::endl;
    std::string frame_id = cad_mesh_in.header.frame_id;
    cad_percept::cgal::msgToMeshModel(cad_mesh_in.mesh, &reference_mesh);

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
    cad_percept::cgal::Transformation ctransformation;
    cad_percept::cgal::tfTransformationToCGALTransformation(transform, ctransformation);
    reference_mesh->transform(ctransformation);

    std::cout << "CAD ready" << std::endl;
    got_CAD = true;
  }
}