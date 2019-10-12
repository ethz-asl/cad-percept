#ifndef CPT_UTILS_H
#define CPT_UTILS_H

#include <kindr/minimal/quat-transformation-gtsam.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_definitions/mesh_model.h>
#include <cgal_conversions/mesh_conversions.h>
#include <cgal_conversions/eigen_conversions.h>
#include "cpt_utils/pc_processing.h"

namespace cad_percept {
namespace cpt_utils {

typedef kindr::minimal::QuatTransformationTemplate<double> SE3;
typedef SE3::Rotation SO3;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

struct Associations {
  Eigen::Matrix3Xd points_from; // dynamic, double
  Eigen::Matrix3Xd points_to;
  Eigen::VectorXd distances;
  Eigen::VectorXd triangles_to; // triangle ID association
};

/**
 * Computes closest Point on Plane for a given point
 */
cgal::Point closestPointOnPlane(const cgal::Plane &plane, const cgal::Point &point);

/**
 * Computes intersection point between a plane and a line
 */
void intersection(const cgal::Plane &plane, const cgal::Line &line, cgal::Point *point);

cgal::Vector getNormalFromPlane(const cgal::Plane &plane);

// Associate point-cloud with architect model.
Associations associatePointCloud(const PointCloud &pc_msg, cgal::MeshModel *mesh_model);

cgal::Point centerOfBbox(const CGAL::Bbox_3 &bbox);
cgal::Point centerOfBbox(const PointCloud &pointcloud);

/**
 *  Estimate of bbox diameter
 */
void bboxDiameters(const CGAL::Bbox_3 bbox, double *width, double *height);

}
}

#endif // CPT_UTILS_H