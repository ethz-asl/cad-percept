#ifndef CGAL_VISUALIZATIONS_TRIANGLE_MESH_DISPLAY_H
#define CGAL_VISUALIZATIONS_TRIANGLE_MESH_DISPLAY_H

#include <cgal_msgs/TriangleMeshStamped.h>
#include <rviz/message_filter_display.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <Eigen/Dense>
#include "mesh_visual.h"

namespace cad_percept {

namespace visualizations {

class MeshVisual;  // Forward definition

class TriangleMeshDisplay
    : public rviz::MessageFilterDisplay<cgal_msgs::TriangleMeshStamped> {
  Q_OBJECT
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  TriangleMeshDisplay() {}
  virtual ~TriangleMeshDisplay();

  void initProperties();

 private Q_SLOTS:
  // Property Callbacks
  void backfaceCullingPropertyChanged();
  void appearencePropertyChanged();

 protected:
  virtual void onInitialize();
  virtual void reset();

 private:
  void processMessage(const cgal_msgs::TriangleMeshStamped::ConstPtr& msg);

  struct {
    rviz::BoolProperty* BackfaceCulling;
    rviz::ColorProperty* SurfaceColor;
    rviz::ColorProperty* EdgeColor;
    rviz::FloatProperty* Alpha;

  } properties_;

  std::unique_ptr<MeshVisual> visual_;
};

}  // namespace visualizations
}  // namespace cad_percept

#endif  // CGAL_VISUALIZATIONS_TRIANGLE_MESH_DISPLAY_H
