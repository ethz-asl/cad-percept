#ifndef CGAL_VISUALIZATIONS_MESH_DISPLAY_H_
#define CGAL_VISUALIZATIONS_MESH_DISPLAY_H_

#include <cgal_msgs/TriangleMeshStamped.h>
#include <rviz/message_filter_display.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <Eigen/Dense>
#include "mesh_visual.h"
#include "q_mesh_display.h"

namespace cad_percept {
namespace visualizations {

class MeshVisual;  // Forward definition

/*
 * Templated MeshDisplay Class
 * ONLY compiles if
 *  a) T is a valid ROS Message Type with Header
 *  b) MeshVisual has an overload to handle whatever T is.
 *
 *  There's a bit of a peculiarity in the implementation of this.
 *  - QMeshDisplay handles all QT special things (implementation of Q_Object!)
 *    As Q_Objects cannot be templated, this is handled in a separate abstract base that is not
 *    templated.
 *
 *  - MeshDisplay then inherits from TWO classes, MessageFilterDisplay for all the RVIZ things, and
 *    QMeshDisplay for all QT specific stuff.
 *
 *  One of the few occurences where multiple inheritance really solves a problem ;-) So MeshDisplay
 *  has the behaviour of both independent base classes.
 */
template <class T>
class MeshDisplay : public rviz::MessageFilterDisplay<T>, public QMeshDisplay {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MeshDisplay();
  virtual ~MeshDisplay();

  void initProperties();

 protected:
  // Property Callbacks
  void backfaceCullingPropertyChanged() final;
  void appearencePropertyChanged() final;

  virtual void onInitialize();
  virtual void reset();

 private:
  void updateVisualProperties();
  void processMessage(const typename T::ConstPtr& msg);

  struct {
    rviz::BoolProperty* BackfaceCulling;
    rviz::ColorProperty* SurfaceColor;
    rviz::ColorProperty* EdgeColor;
    rviz::FloatProperty* Alpha;

  } properties_;
  std::unique_ptr<MeshVisual> visual_;
};

/*
 * That's where the magic happens -
 *   MeshDisplay typedefd with the different Message types in order to obtain
 *   the exportable, pluginlib compatible full implementations.
 */
typedef MeshDisplay<cgal_msgs::TriangleMeshStamped> TriangleMeshDisplay;
typedef MeshDisplay<cgal_msgs::ProbabilisticMesh> ProbabilisticMeshDisplay;
typedef MeshDisplay<cgal_msgs::ColoredMesh> ColoredMeshDisplay;

}  // namespace visualizations
}  // namespace cad_percept

#endif  // CGAL_VISUALIZATIONS_MESH_DISPLAY_H_
