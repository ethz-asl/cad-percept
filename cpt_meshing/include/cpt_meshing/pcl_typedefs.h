#ifndef CPT_MESHING_PCL_TYPEDEFS_H
#define CPT_MESHING_PCL_TYPEDEFS_H

#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>

namespace cad_percept {
namespace meshing {

typedef pcl::PointXYZ InputPoint;
typedef pcl::PointCloud<InputPoint> InputPointCloud;
typedef pcl::PointCloud<pcl::Normal> InputNormals;

typedef pcl::Filter<InputPoint> InputPointCloudFilter;
}  // namespace meshing
}  // namespace cad_percept
#endif  // CPT_MESHING_PCL_TYPEDEFS_H
