#ifndef CPT_MESHING_PCL_TYPEDEFS_H
#define CPT_MESHING_PCL_TYPEDEFS_H

#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>

namespace cad_percept {
namespace meshing {

typedef pcl::PointXYZ InputPoint;
typedef pcl::PointCloud<InputPoint> InputPointCloud;
typedef pcl::PointCloud<pcl::Normal> InputNormals;

typedef pcl::Filter<InputPoint> InputPointCloudFilter;

}
}
#endif //CPT_MESHING_PCL_TYPEDEFS_H
