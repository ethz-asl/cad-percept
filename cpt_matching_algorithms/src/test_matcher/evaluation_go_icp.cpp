#include <ros/ros.h>
#include <chrono>
#include <random>

#include "plane_extraction/plane_extraction.h"
#include "plane_matching/plane_matching.h"
#include "test_matcher/go_icp_matcher.h"

using namespace cad_percept;
using namespace matching_algorithms;

int main(int argc, char** argv) {
  ros::init(argc, argv, "matcher_test");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::cout << "///////////////////////////////////////////////" << std::endl;
  std::cout << "             GoICP Evaluation started          " << std::endl;
  std::cout << "///////////////////////////////////////////////" << std::endl;

  bool useGoICP = nh_private.param<bool>("useEvalGoICP", true);
  bool makeGoICPTest = nh_private.param<bool>("makeGoICPTest", true);
  std::string goicp_location = nh_private.param<std::string>("goicp_folder", "fail");
  int max_test_iterations = nh_private.param<int>("maxTestIter", 1);
  std::string test_result_file = nh_private.param<std::string>("test_results", "fail");
  std::string cache_folder = nh_private.param<std::string>("cache_folder", "fail");
  std::string path_to_construct = nh_private.param<std::string>("construct_file", "fail");

  Eigen::Matrix4d res_transform;
  Eigen::Matrix4d gt_transform;
  ros::Publisher scan_pub = nh.advertise<sensor_msgs::PointCloud2>("scan", 1);
  ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>("map", 1);

  // Load mesh
  MeshModel::Ptr model;
  model->create(path_to_construct, &model, false);

  pcl::PointCloud<pcl::PointXYZ> map_point_cloud;
  map_point_cloud.header.frame_id = "map";

  cad_percept::cpt_utils::sample_pc_from_mesh(model->getMesh(), 1000, 0.0, &map_point_cloud);

  // Sample from mesh (noise ?)
  pcl::PointCloud<pcl::PointXYZ> scan_point_cloud;
  scan_point_cloud.header.frame_id = "map";
  pcl::copyPointCloud(map_point_cloud, scan_point_cloud);

  std::ofstream actuel_file(test_result_file);

  // Make a short test using the demo
  // Read demo examples
  if (useGoICP && makeGoICPTest) {
    pcl::PointCloud<pcl::PointXYZ> data_bunny;
    pcl::PointCloud<pcl::PointXYZ> model_bunny;

    chdir(goicp_location.c_str());
    pcl::PointXYZ demo_points;
    std::ifstream demo_files;
    int number_demo_points;
    demo_files.open("model_bunny.txt");
    demo_files >> number_demo_points;
    for (int point = 0; point < number_demo_points; ++point) {
      demo_files >> demo_points.x;
      demo_files >> demo_points.y;
      demo_files >> demo_points.z;
      data_bunny.push_back(demo_points);
    }
    Eigen::Matrix4f transform_demo = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f rotation_demo = (Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()) *
                                     Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitY()) *
                                     Eigen::AngleAxisf(M_PI * 4 / 7, Eigen::Vector3f::UnitZ()))
                                        .matrix();
    Eigen::Vector3f translation_demo = Eigen::Vector3f(5, 2, 3);
    transform_demo.block(0, 0, 3, 3) = rotation_demo;
    transform_demo.block(0, 3, 3, 1) = translation_demo;
    pcl::transformPointCloud(data_bunny, data_bunny, transform_demo);
    demo_files.close();
    demo_files.open("model_bunny.txt");
    demo_files >> number_demo_points;
    for (int point = 0; point < number_demo_points; ++point) {
      demo_files >> demo_points.x;
      demo_files >> demo_points.y;
      demo_files >> demo_points.z;
      model_bunny.push_back(demo_points);
    }
    demo_files.close();

    GoIcp::goIcpMatch(res_transform, data_bunny, model_bunny);

    // Transform
    pcl::transformPointCloud(data_bunny, data_bunny, res_transform);
    std::cout << res_transform << std::endl;

    model_bunny.header.frame_id = "map";
    data_bunny.header.frame_id = "map";
    scan_pub.publish(data_bunny.makeShared());
    map_pub.publish(model_bunny.makeShared());

    ros::spinOnce();
    std::cout << "Demo processed, press enter to continue..." << std::endl;
    std::cin.ignore();
  }

  map_pub.publish(map_point_cloud.makeShared());

  // Evaluation
  pcl::PointCloud<pcl::PointXYZ> transformed_scan;
  srand(time(0));
  transformed_scan.header.frame_id = "map";

  std::chrono::duration<int, std::milli> duration;
  std::chrono::steady_clock::time_point t_start;
  std::chrono::steady_clock::time_point t_end;
  Eigen::Vector3d gt_translation;
  Eigen::Quaterniond gt_rotation;
  Eigen::Quaterniond calc_rotation;
  Eigen::Matrix<double, 7, 1> transform_TR;
  float translation_error = 0;
  float rotation_error = 0;
  float transform_error = 0;

  // Load map for PRRUS
  BoundedPlanes* map_planes = new BoundedPlanes(*model, cache_folder);
  cgal::Transformation cgal_transform;
  std::vector<Eigen::Vector3d> plane_normals;
  std::vector<pcl::PointCloud<pcl::PointXYZ>> extracted_planes;
  pcl::PointCloud<pcl::PointNormal> scan_planes;

  for (int i = 0; i < max_test_iterations; i++) {
    gt_transform(0, 3) = 0;
    gt_transform(1, 3) = 0;
    // Sample along z-axis (x, y fixed to largest vertical axis inside construct), rotation
    // uniform
    gt_transform(2, 3) = (double)(std::rand() % 10) * 0.1 + 0.5;
    // Uniform sampling
    gt_transform.block(0, 0, 3, 3) = (Eigen::Matrix3d)(
        Eigen::AngleAxisd((std::rand() % 30) * M_PI / 15 - M_PI / 2, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd((std::rand() % 30) * M_PI / 15 - M_PI / 2, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd((std::rand() % 30) * M_PI / 15 - M_PI / 2, Eigen::Vector3d::UnitZ()));

    pcl::transformPointCloud(scan_point_cloud, transformed_scan, gt_transform);

    if (useGoICP) {
      t_start = std::chrono::steady_clock::now();
      GoIcp::goIcpMatch(res_transform, transformed_scan, map_point_cloud);
      t_end = std::chrono::steady_clock::now();

      pcl::transformPointCloud(transformed_scan, transformed_scan, res_transform);
      scan_pub.publish(transformed_scan.makeShared());
    } else {  // use PRRUS
      t_start = std::chrono::steady_clock::now();

      PlaneExtractor::iterRhtPlaneExtraction(extracted_planes, plane_normals, transformed_scan,
                                             PlaneExtractor::loadIterRhtConfigFromServer());
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
        scan_planes.push_back(norm_point);
        plane_nr++;
      }
      transform_error = PlaneMatch::prrus(cgal_transform, scan_planes, *map_planes,
                                          PlaneMatch::loadPrrusConfigFromServer());
      cgal::cgalTransformationToEigenTransformation(cgal_transform, &res_transform);
      t_end = std::chrono::steady_clock::now();

      int i = 0;
      for (auto extracted_plane : extracted_planes) {
        extracted_planes[i].clear();
        pcl::transformPointCloud(extracted_plane, extracted_planes[i], res_transform);
        i++;
      }
      PlaneExtractor::visualizePlane(extracted_planes, scan_pub, "map");
      extracted_planes.clear();
      plane_normals.clear();
      scan_planes.clear();
    }
    ros::spinOnce();

    // Write to testresults
    gt_translation = Eigen::Vector3d(gt_transform(0, 3), gt_transform(1, 3), gt_transform(2, 3));
    gt_rotation = Eigen::Quaterniond((Eigen::Matrix3d)gt_transform.block(0, 0, 3, 3));
    transform_TR(0, 0) = res_transform(0, 3);
    transform_TR(1, 0) = res_transform(1, 3);
    transform_TR(2, 0) = res_transform(2, 3);
    calc_rotation = Eigen::Quaterniond((Eigen::Matrix3d)res_transform.block(0, 0, 3, 3));
    transform_TR(3, 0) = calc_rotation.w();
    transform_TR(4, 0) = calc_rotation.x();
    transform_TR(5, 0) = calc_rotation.y();
    transform_TR(6, 0) = calc_rotation.z();

    translation_error =
        (Eigen::Vector3d(transform_TR(0, 0), transform_TR(1, 0), transform_TR(2, 0)) -
         gt_translation)
            .norm();
    rotation_error =
        Eigen::AngleAxis<double>(Eigen::Quaterniond(transform_TR(3, 0), transform_TR(4, 0),
                                                    transform_TR(5, 0), transform_TR(6, 0))
                                     .toRotationMatrix() *
                                 gt_rotation.inverse().toRotationMatrix())
            .angle();

    duration = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start);

    actuel_file << gt_translation[0] << " " << gt_translation[1] << " " << gt_translation[2] << " "
                << gt_rotation.w() << " " << gt_rotation.x() << " " << gt_rotation.y() << " "
                << gt_rotation.z() << " " << transform_TR[0] << " " << transform_TR[1] << " "
                << transform_TR[2] << " " << transform_TR[3] << " " << transform_TR[4] << " "
                << transform_TR[5] << " " << transform_TR[6] << " " << translation_error << " "
                << rotation_error << " " << duration.count() << " " << transform_error << std::endl;
  }
  actuel_file.close();

  return 0;
}