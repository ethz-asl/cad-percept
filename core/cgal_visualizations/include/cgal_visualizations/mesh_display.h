#ifndef CGAL_VISUALIZATIONS_MESH_DISPLAY_H_
#define CGAL_VISUALIZATIONS_MESH_DISPLAY_H_

#include <cgal_msgs/TriangleMeshStamped.h>
#include <cgal_visualizations/mesh_visual.h>
#include <rviz/message_filter_display.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <Eigen/Dense>

namespace cad_percept {
namespace visualizations {

class MeshVisual;  // Forward definition

/*
 * Templated MeshDisplay Class
 * ONLY compiles if
 *  a) T is a valid ROS Message Type with Header
 *  b) MeshVisual has an overload to handle whatever T is.
 *
 *   Note that MeshDisplay is not a valid QObject (yet) to be used in RVIZ.
 *   This happens in the explicit template instantiations in
 *   mesh_display_instantiations.h
 */
template <class T>
class MeshDisplay : public rviz::MessageFilterDisplay<T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MeshDisplay();
  ~MeshDisplay();

  void initProperties();

 protected:
  // Property Callbacks
  virtual void backfaceCullingPropertyChanged();
  virtual void appearencePropertyChanged();

  void onInitialize() override;
  void reset() override;

  // regular update from rviy::Display
  void update(float wall_dt, float ros_dt) override;

  void updateVisualProperties();

  // processes new messages
  void processMessage(const typename T::ConstPtr& msg) override;

  // updates model position, returns true if the position changed.
  bool updateTF();

  struct {
    rviz::BoolProperty* BackfaceCulling;
    rviz::ColorProperty* SurfaceColor;
    rviz::ColorProperty* EdgeColor;
    rviz::FloatProperty* Alpha;

  } properties_;
  std::unique_ptr<MeshVisual> visual_;

  // utilities for tracking the tf state
  std::string parent_frame_;
  Ogre::Quaternion orientation_;
  Ogre::Vector3 position_;

  // message_filters::Subscriber<T> tf_sub_;
  // tf2_ros::MessageFilter<T>* parent_tf_filter_;
};

}  // namespace visualizations
}  // namespace cad_percept

#endif  // CGAL_VISUALIZATIONS_MESH_DISPLAY_H_
