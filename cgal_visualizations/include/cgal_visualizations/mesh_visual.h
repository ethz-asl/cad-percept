/**
 * This is a general class for all mesh visualizations and can be included into
 * the rviz plugins.
 */

#ifndef TRIANGLE_MESH_VISUALIZATIONS_H
#define TRIANGLE_MESH_VISUALIZATIONS_H

#include <OGRE/OgreManualObject.h>
#include <cgal_msgs/ColoredMesh.h>
#include <cgal_msgs/ProbabilisticMesh.h>
#include <cgal_msgs/TriangleMesh.h>
#include <cgal_msgs/TriangleMeshStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/ColorRGBA.h>
#include <QtGui/QColor>

namespace cad_percept {
namespace visualizations {

class MeshVisual {
 public:
  MeshVisual(Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node);
  virtual ~MeshVisual();

  void initResourcePaths();
  void setMessage(const cgal_msgs::TriangleMeshStamped::ConstPtr &msg);
  void setMessage(const cgal_msgs::ColoredMesh::ConstPtr &msg);
  void setMessage(const cgal_msgs::ProbabilisticMesh::ConstPtr &msg);

  // Set the coordinate frame pose.
  void setFramePosition(const Ogre::Vector3 &position);
  void setFrameOrientation(const Ogre::Quaternion &orientation);

  void update();

  void clear();

  void setBackFaceCulling(const bool value) { backface_culling_ = value; }

  void setAppearance(const Ogre::ColourValue edge_color, const Ogre::ColourValue surface_color,
                     const float alpha) {
    std_edge_color_ = edge_color;
    std_edge_color_.a = alpha;

    std_surface_color_ = surface_color;
    std_surface_color_.a = alpha;
  }

 private:
  Ogre::SceneNode *frame_node_;
  Ogre::SceneManager *scene_manager_;
  std::string object_name_;
  cgal_msgs::TriangleMesh triangle_mesh_msg_;
  std_msgs::ColorRGBA mesh_color_msg_;
  std::vector<std_msgs::ColorRGBA> surface_colors_msg_;
  std::vector<geometry_msgs::Point> cov_vertices_msg_;
  std::vector<geometry_msgs::Vector3> normals_msg_;

  unsigned int instance_number_;
  static unsigned int instance_counter_;
  bool visualize_covariances_;
  bool backface_culling_;
  bool visualize_color_;
  Ogre::ColourValue std_edge_color_;
  Ogre::ColourValue std_surface_color_;
};
}  // namespace visualizations
}  // namespace cad-percept

#endif  // TRIANGLE_MESH_VISUALIZATIONS_H
