#include <cpt_meshing/mesher/delaunay_3d_mesher.h>
#include <cpt_meshing/preprocessing/pre_processing_filter.h>
#include <cpt_meshing/pcl_typedefs.h>
#include <cgal_definitions/cgal_typedefs.h>
#include <iostream>
using namespace std;

int main() {
  cad_percept::meshing::Delaunay3DMesher mesher;
  cad_percept::meshing::PreProcessingFilter preprocessing;

  preprocessing.addVoxelFilter(0.02, 0.02, 0.02);
  preprocessing.addBoxFilter("x", -2.0, 2.0);
  preprocessing.addBoxFilter("y", -2.0, 2.0);
  preprocessing.addBoxFilter("z", 0.2, 4.0);

  cad_percept::meshing::InputPointCloud::Ptr cloud;
  cad_percept::meshing::InputPointCloud::Ptr cloud_filtered;
  cad_percept::meshing::InputNormals::Ptr normals;

  preprocessing.run(cloud, cloud_filtered, normals);
  mesher.addPointCloud(cloud_filtered, normals);

  cad_percept::cgal::Polyhedron mesh;
  mesher.getMesh(&mesh);

  cout << "Hello, World!";
  return 0;
}