#include "cpt_utils/pc_processing.h"

#include <cpt_utils/cpt_utils.h>

namespace cad_percept {
namespace cpt_utils {

/**
 *  This function was not tested yet!
 *  (mpantic, Aug 2020): Removed, should be moved somewhere else to avoid libpointmatcher dependency

void align_sequence(const boost::circular_buffer<PointCloud> &cb, PointCloud *pointcloud_out) {
  PM::ICP icp;
  icp.setDefault();
  DP mapPointCloud, newCloud;
  TP T_to_map_from_new = TP::Identity(4, 4);

  // Rigid transformation
  std::shared_ptr<PM::Transformation> rigidTrans;
  rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");

  // Create filters manually to clean the global map
  std::shared_ptr<PM::DataPointsFilter> densityFilter = PM::get().DataPointsFilterRegistrar.create(
      "SurfaceNormalDataPointsFilter",
      {{"knn", "10"}, {"epsilon", "5"}, {"keepNormals", "0"}, {"keepDensities", "1"}});

  std::shared_ptr<PM::DataPointsFilter> maxDensitySubsample =
      PM::get().DataPointsFilterRegistrar.create(
          "MaxDensityDataPointsFilter", {{"maxDensity", PointMatcherSupport::toParam(30)}});

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
    } catch (PM::ConvergenceError &error) {
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
  // alternatively rosMsgToPointMatcherCloud (libpointmatcher_ros)
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

  DP dppointcloud = DP(
      feat, featLabels);  // construct a point cloud from existing features without any descriptor

  return dppointcloud;
}
*/

void transformPointCloud(PointCloud *pointcloud, const Eigen::Affine3f &transform) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*pointcloud, *pointcloud, transform);
}

void sample_pc_from_mesh(const cgal::Polyhedron &P, const int no_of_points, const double stddev,
                         PointCloud *pointcloud) {
  CHECK(pointcloud);

  // Sample points from mesh
  std::vector<cgal::Point> points_noise;
  samplePointsFromMesh(P, no_of_points, stddev, &points_noise);

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
}

/*
void projectToPlane(const PointCloud &cloud_in, const cgal::ShapeKernel::Plane_3 &plane,
                    PointCloud *cloud_out) {
  for (auto point : cloud_in) {
    cgal::ShapeKernel::Point_3 p_proj;
    p_proj = plane.projection(cgal::ShapeKernel::Point_3(point.x, point.y, point.z));
    cloud_out->push_back(pcl::PointXYZ(p_proj.x(), p_proj.y(), p_proj.z()));
  }
}
*/
void projectToPlane(const PointCloud &cloud_in, const cgal::Plane &plane, PointCloud *cloud_out) {
  for (auto point : cloud_in) {
    cgal::Point p_proj;
    p_proj = plane.projection(cgal::Point(point.x, point.y, point.z));
    cloud_out->push_back(pcl::PointXYZ(p_proj.x(), p_proj.y(), p_proj.z()));
  }
}

void projectToPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
                    const pcl::ModelCoefficients::Ptr coefficients,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out) {
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(cloud_in);
  proj.setModelCoefficients(coefficients);
  proj.filter(*cloud_out);
}

double getArea(const PointCloud &pointcloud) {
  // http://pointclouds.org/documentation/tutorials/convex_hull_2d.php

  // copy pointer
  pcl::PointCloud<pcl::PointXYZ>::Ptr mycloudPtr;
  mycloudPtr = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >(
      new pcl::PointCloud<pcl::PointXYZ>(pointcloud));

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ConvexHull<pcl::PointXYZ> chull;
  chull.setDimension(2);
  chull.setInputCloud(mycloudPtr);
  // set hull to 2D. If this fails, projectToPlane was not executed before
  chull.setComputeAreaVolume(true);
  chull.reconstruct(*cloud_hull);
  return chull.getTotalArea();
}

void computePCBbox(const PointCloud &pointcloud, CGAL::Bbox_3 *bbox) {
  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(pointcloud, min_pt, max_pt);
  *bbox = CGAL::Bbox_3(min_pt.x, min_pt.y, min_pt.z, max_pt.x, max_pt.y, max_pt.z);
}

void bboxDiameters(const PointCloud &pointcloud, double *width, double *height) {
  pcl::PointXYZ minPoint, maxPoint;
  pcl::getMinMax3D(pointcloud, minPoint, maxPoint);
  *width = sqrt(pow(maxPoint.x - minPoint.x, 2) + pow(maxPoint.y - minPoint.y, 2));
  *height = sqrt(maxPoint.z - minPoint.z);
}

void removeOutliers(PointCloud *pointcloud, int knn, float thresh_mult) {
  // copy pointer
  pcl::PointCloud<pcl::PointXYZ>::Ptr mycloudPtr;
  mycloudPtr = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >(
      new pcl::PointCloud<pcl::PointXYZ>(*pointcloud));

  // std::cout << "Cloud before statistical outlier removal: " << mycloudPtr->size() << std::endl;
  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(mycloudPtr);
  sor.setMeanK(knn);
  sor.setStddevMulThresh(thresh_mult);
  sor.filter(*pointcloud);
  // std::cout << "Cloud after statistical outlier removal: " << pointcloud->size() << std::endl;
}

}  // namespace cpt_utils
}  // namespace cad_percept
