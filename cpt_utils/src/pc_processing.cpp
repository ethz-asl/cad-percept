#include "cpt_utils/pc_processing.h"

namespace cad_percept {
namespace cpt_utils {

/**
 *  This function was not tested yet!
 */
void align_sequence(const boost::circular_buffer<PointCloud> &cb, PointCloud *pointcloud_out) {
  PM::ICP icp;
  icp.setDefault();
  DP mapPointCloud, newCloud;
  TP T_to_map_from_new = TP::Identity(4,4);

  // Rigid transformation
  std::shared_ptr<PM::Transformation> rigidTrans;
  rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");

  // Create filters manually to clean the global map
	std::shared_ptr<PM::DataPointsFilter> densityFilter =
					PM::get().DataPointsFilterRegistrar.create(
						"SurfaceNormalDataPointsFilter",
						{
							{"knn", "10"},
							{"epsilon", "5"},
							{"keepNormals", "0"},
							{"keepDensities", "1"}
						}
          );

	std::shared_ptr<PM::DataPointsFilter> maxDensitySubsample =
					PM::get().DataPointsFilterRegistrar.create(
						"MaxDensityDataPointsFilter",
						{{"maxDensity", PointMatcherSupport::toParam(30)}}
          );

  int i = 0;
  for (auto pointcloud : cb) {
    std::cout << "Loading PointCloud for alignment " << i << std::endl;
    newCloud = pointCloudToDP(pointcloud);

    if (mapPointCloud.getNbPoints() == 0) {
      mapPointCloud = newCloud;
      continue;
    }

    // call ICP
    try {
      // use last transformation as a prior makes no sense in map frame
      // const TP prior = T_to_map_from_new;
      T_to_map_from_new = icp(newCloud, mapPointCloud);
    }
    catch (PM::ConvergenceError &error) {
      std::cout << "ERROR PM::ICP alignment failed to converge: " << std::endl;
      std::cout << " " << error.what() << std::endl;
      continue;
    }

    // force orthogonality of the rotation matrix
    T_to_map_from_new = rigidTrans->correctParameters(T_to_map_from_new);

    // move the new point cloud
    newCloud = rigidTrans->compute(newCloud, T_to_map_from_new);

    // merge point clouds to map
    mapPointCloud.concatenate(newCloud);

    // clean the map
    mapPointCloud = densityFilter->filter(mapPointCloud);
    mapPointCloud = maxDensitySubsample->filter(mapPointCloud);
  }
  *pointcloud_out = dpToPointCloud(mapPointCloud);
  mapPointCloud.save("/home/julian/megabot_ws/src/cad-percept/srd_relative_deviations/resources/pc_align_sequence.pcd");
}

PointCloud dpToPointCloud(const DP &dppointcloud) {
  const size_t n_points = dppointcloud.getNbPoints();
  PointCloud cloud;
  for (uint i = 0; i < n_points; ++i) {
    pcl::PointXYZ point;
    point.x = dppointcloud.features(0, i);
    point.y = dppointcloud.features(1, i);
    point.z = dppointcloud.features(2, i);
    cloud.push_back(point);
  }
  return cloud;
}

DP pointCloudToDP(const PointCloud &pointcloud) {
  // alternatively use matrix3dEigenToPointMatcher (octomap_compare)
  // or rosMsgToPointMatcherCloud (libpointmatcher_ros)
  const int dimFeatures = 4;

  PM::Matrix feat(dimFeatures, pointcloud.points.size());
  for (uint i = 0; i < pointcloud.points.size(); ++i) {
    feat(0, i) = pointcloud[i].x;
    feat(1, i) = pointcloud[i].y;
    feat(2, i) = pointcloud[i].z;
    feat(3, i) = 1.0;
  }

  DP::Labels featLabels;
  featLabels.push_back(DP::Label("x", 1));
  featLabels.push_back(DP::Label("y", 1));
  featLabels.push_back(DP::Label("z", 1));
  featLabels.push_back(DP::Label("pad", 1));

  DP dppointcloud = DP(feat, featLabels); // construct a point cloud from existing features without any descriptor

  return dppointcloud;
}

void transformPointCloud(PointCloud *pointcloud, const Eigen::Affine3f &transform) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::transformPointCloud (*pointcloud, *pointcloud, transform);
}

void sample_pc_from_mesh(const cgal::Polyhedron &P, 
                         const int no_of_points,
                         const double stddev,
                         PointCloud *pointcloud,
                         std::string file_name) {
  // generate random point sets on triangle mesh
  std::vector<cgal::Point> points;
  // Create the generator, input is the Polyhedron P
  CGAL::Random_points_in_triangle_mesh_3<cgal::Polyhedron> g(P);
  // Get no_of_points random points in cdt
  CGAL::cpp11::copy_n(g, no_of_points, std::back_inserter(points));
  // Check that we have really created no_of_points points.
  assert(points.size() == no_of_points);
  // print the first point that was generated
  //std::cout << points[0] << std::endl;  

  // add random noise with Gaussian distribution
  const double mean = 0.0;
  std::default_random_engine generator;
  std::normal_distribution<double> dist(mean, stddev);

  std::vector<cgal::Point> points_noise;
  for (auto point : points) {
    cgal::Point p_noise;
    p_noise = cgal::Point(point.x() + dist(generator),
                    point.y() + dist(generator),
                    point.z() + dist(generator));
    points_noise.push_back(p_noise);
  }

  // save as pcd
  pointcloud->width = points_noise.size();
  pointcloud->height = 1;
  pointcloud->header.frame_id = "mesh";
  pointcloud->is_dense = false;

  for (auto point : points_noise) {
    pcl::PointXYZ cloudpoint;
    cloudpoint.x = (float)point.x();
    cloudpoint.y = (float)point.y();
    cloudpoint.z = (float)point.z();
    pointcloud->push_back(cloudpoint);
  }

  //std::stringstream ss;
  //ss << "/home/julian/megabot_ws/src/cad-percept/relative_deviations/resources/" << file_name << ".pcd";
  //pcl::io::savePCDFileASCII(ss.str(), *pointcloud);
  //std::cerr << "Saved " << pointcloud->points.size() << " data points to pcd" << std::endl;
}

}
}