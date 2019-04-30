#ifndef CPT_MESHING_PCL_TYPEDEFS_H
#define CPT_MESHING_PCL_TYPEDEFS_H

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

namespace cad_percept {
namespace meshing {

typedef pcl::PointCloud<pcl::PointXYZ> InputPointCloud;
typedef pcl::PointCloud<pcl::Normal> InputNormals;

}
}
#endif //CPT_MESHING_PCL_TYPEDEFS_H
