#ifndef CGAL_VISUALIZATIONS_PROBABILISTIC_MESH_VISUAL_H
#define CGAL_VISUALIZATIONS_PROBABILISTIC_MESH_VISUAL_H

#include <OGRE/OgreManualObject.h>
#include <cgal_msgs/ProbabilisticMesh.h>

namespace cad_percept {
namespace visualizations {

class ProbabilisticMeshVisual {
 public:
  ProbabilisticMeshVisual(Ogre::SceneManager* scene_manager,
                          Ogre::SceneNode* parent_node);
  virtual ~ProbabilisticMeshVisual();

  void setMessage(const cgal_msgs::ProbabilisticMesh::ConstPtr& msg);

  // Set the coordinate frame pose.
  void setFramePosition(const Ogre::Vector3& position);
  void setFrameOrientation(const Ogre::Quaternion& orientation);

 private:
  Ogre::SceneNode* frame_node_;
  Ogre::SceneManager* scene_manager_;
  std::string object_name_;
  unsigned int instance_number_;
  static unsigned int instance_counter_;
  bool visualize_covariances_;
};
} // namespace visualizations
}  // namespace cad-percept

#endif  // CGAL_VISUALIZATIONS_PROBABILISTIC_MESH_VISUAL_H
