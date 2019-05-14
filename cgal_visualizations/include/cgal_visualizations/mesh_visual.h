#ifndef CGAL_VISUALIZATIONS_PROBABILISTIC_MESH_VISUAL_H
#define CGAL_VISUALIZATIONS_PROBABILISTIC_MESH_VISUAL_H

#include <OGRE/OgreManualObject.h>
#include <cgal_msgs/TriangleMesh.h>
#include <QtGui/QColor>

namespace cad_percept {
namespace visualizations {

class MeshVisual {
 public:
  MeshVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);
  virtual ~MeshVisual();

  void initResourcePaths();
  void setMessage(const cgal_msgs::TriangleMesh& msg);

  // Set the coordinate frame pose.
  void setFramePosition(const Ogre::Vector3& position);
  void setFrameOrientation(const Ogre::Quaternion& orientation);

  void update();

  void clear();

  void setBackFaceCulling(const bool value) { backface_culling_ = value; }

  void setAppearance(const Ogre::ColourValue edge_color,
                     const Ogre::ColourValue surface_color, const float alpha) {
    std_edge_color_ = edge_color;
    std_edge_color_.a = alpha;

    std_surface_color_ = surface_color;
    std_surface_color_.a = alpha;
  }

 private:
  Ogre::SceneNode* frame_node_;
  Ogre::SceneManager* scene_manager_;
  std::string object_name_;
  cgal_msgs::TriangleMesh triangle_mesh_msg_;

  unsigned int instance_number_;
  static unsigned int instance_counter_;
  bool visualize_covariances_;
  bool per_vertex_color_;
  bool backface_culling_;
  std::vector<Ogre::ColourValue> surface_colors_;
  Ogre::ColourValue std_edge_color_;
  Ogre::ColourValue std_surface_color_;
};
}  // namespace visualizations
}  // namespace cad-percept

#endif  // CGAL_VISUALIZATIONS_PROBABILISTIC_MESH_VISUAL_H
