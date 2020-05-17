#include <ros/ros.h>

#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_definitions/mesh_model.h>
#include <cgal_msgs/TriangleMeshStamped.h>
#include <cpt_utils/pc_processing.h>
#include <chrono>
#include <random>

#include "cloud_filter/cloud_filter.h"
#include "plane_extraction/plane_extraction.h"

using namespace cad_percept;
using namespace matching_algorithms;
using namespace cgal;

template <class HDS>
class TestingMesh : public CGAL::Modifier_base<HDS> {
 public:
  TestingMesh() {}
  void operator()(HDS& hds) {
    // Create a simple plane parallel to y - z - plane at x = 10
    CGAL::Polyhedron_incremental_builder_3<HDS> B(hds, true);

    B.begin_surface(3, 3);
    B.add_vertex(Point(10, 10, 10));
    B.add_vertex(Point(10, -10, 10));
    B.add_vertex(Point(-10, -10, 10));

    B.begin_facet();
    B.add_vertex_to_facet(0);
    B.add_vertex_to_facet(1);
    B.add_vertex_to_facet(2);
    B.end_facet();

    B.end_surface();
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_plane_extraction");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  ros::Publisher plane_pub_;

  std::cout << "/// Started test plane extraction ///" << std::endl;

  // Create mesh cube
  Polyhedron m;
  TestingMesh<HalfedgeDS> testcase;
  m.delegate(testcase);

  MeshModel::Ptr model;
  MeshModel::create(m, &model);

  pcl::PointCloud<pcl::PointXYZ> pointcloud;
  cad_percept::cpt_utils::sample_pc_from_mesh(model->getMesh(), 1000, 0.0, &pointcloud);

  // Run test iterations
  std::chrono::steady_clock::time_point t_start;
  std::chrono::steady_clock::time_point t_end;
  std::chrono::duration<int, std::milli> duration;

  std::string extractor = nh_private.param<std::string>("PlaneExtractor", "fail");
  int test_iterations = nh_private.param<int>("numTestIterations", 10);
  float search_radius = nh_private.param<float>("search_radius", 0.1);
  srand(time(0));

  Eigen::Matrix4d sample_transform = Eigen::Matrix4d::Identity();
  Eigen::Quaterniond rotation;
  Eigen::Vector3d translation;
  double euler_x;
  double euler_y;
  double euler_z;
  double x;
  double y;
  double z;
  Eigen::Vector3d gt_normal;
  float rotation_error;

  pcl::PointCloud<pcl::PointXYZ> lidar_scan;
  std::vector<Eigen::Vector3d> plane_normals;
  std::vector<pcl::PointCloud<pcl::PointXYZ>> extracted_planes;
  std::string tf_lidar_frame = "map";

  std::cout << "Ready for testing" << std::endl;
  std::string test_result_file = nh_private.param<std::string>("test_results", "fail");
  std::ofstream actuel_file(test_result_file);

  for (int iter = 0; iter < test_iterations; iter++) {
    std::cout << "Start iteration " << iter << std::endl;
    try {
      // Rotate cube
      // // Uniform sampling
      euler_x = (std::rand() % 30) * M_PI / 15 - M_PI / 2;
      euler_y = (std::rand() % 30) * M_PI / 15 - M_PI / 2;
      euler_z = (std::rand() % 30) * M_PI / 15 - M_PI / 2;
      x = (std::rand() % 6000) * 0.01 - 30;
      y = (std::rand() % 6000) * 0.01 - 30;
      z = (std::rand() % 6000) * 0.01 - 30;

      // double euler_x = 0;
      // double euler_y = 0;
      // double euler_z = (std::rand() % 30) * M_PI / 15 - M_PI / 2;

      rotation = Eigen::AngleAxisd(euler_x, Eigen::Vector3d::UnitX()) *
                 Eigen::AngleAxisd(euler_y, Eigen::Vector3d::UnitY()) *
                 Eigen::AngleAxisd(euler_z, Eigen::Vector3d::UnitZ());
      translation = Eigen::Vector3d(x, y, z);
      sample_transform.block(0, 0, 3, 3) = rotation.matrix();
      sample_transform.block(0, 3, 3, 1) = translation;

      pcl::transformPointCloud(pointcloud, lidar_scan, sample_transform);

      t_start = std::chrono::steady_clock::now();
      // Detect planes
      // Filtering / Preprocessing Point Cloud
      if (nh_private.param<bool>("useVoxelCentroidFilter", false)) {
        CloudFilter::filterVoxelCentroid(search_radius, lidar_scan);
      }

      if (lidar_scan.size() == 0) {
        std::cout << "Can not find any planes in scan, as scan is empty after filtering"
                  << std::endl;
        break;
      }
      // Plane Extraction
      extracted_planes.clear();
      plane_normals.clear();
      if (!extractor.compare("pclPlaneExtraction")) {
        PlaneExtractor::pclPlaneExtraction(extracted_planes, plane_normals, lidar_scan,
                                           tf_lidar_frame, plane_pub_);
      } else if (!extractor.compare("rhtPlaneExtraction")) {
        PlaneExtractor::rhtPlaneExtraction(extracted_planes, plane_normals, lidar_scan,
                                           tf_lidar_frame, plane_pub_,
                                           PlaneExtractor::loadRhtConfigFromServer());
      } else if (!extractor.compare("iterRhtPlaneExtraction")) {
        PlaneExtractor::iterRhtPlaneExtraction(extracted_planes, plane_normals, lidar_scan,
                                               tf_lidar_frame, plane_pub_);
      } else if (!extractor.compare("cgalRegionGrowing")) {
        PlaneExtractor::cgalRegionGrowing(extracted_planes, plane_normals, lidar_scan,
                                          tf_lidar_frame, plane_pub_);
      } else {
        std::cout << "Error: Could not find given plane extractor" << std::endl;
        break;
      }
      t_end = std::chrono::steady_clock::now();

      // Evaluation
      gt_normal = rotation * Eigen::Vector3d(0, 0, 1);
      if (gt_normal.dot(plane_normals[0]) < 0) {
        gt_normal = -gt_normal;
      }
      rotation = rotation.FromTwoVectors(plane_normals[0], gt_normal);
      rotation_error = Eigen::AngleAxis<double>(rotation).angle();
      duration = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start);

      actuel_file << gt_normal[0] << " " << gt_normal[1] << " " << gt_normal[2] << " "
                  << plane_normals[0][0] << " " << plane_normals[0][1] << " " << plane_normals[0][2]
                  << " " << rotation_error << " " << duration.count() << std::endl;

    } catch (...) {
      std::cout << "Catch error general" << std::endl;
      std::cin.ignore();
      continue;
    }
  }
  actuel_file.close();

  return 0;
}
