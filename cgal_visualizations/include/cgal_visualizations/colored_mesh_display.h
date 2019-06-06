#ifndef COLORED_MESH_DISPLAY_VISUALIZATIONS_H
#define COLORED_MESH_DISPLAY_VISUALIZATIONS_H

#include <cgal_msgs/ColoredMesh.h>
#include <rviz/message_filter_display.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <Eigen/Dense>
#include "mesh_visual.h"

namespace cad_percept {

namespace visualizations {

class MeshVisual;  // Forward definition

class ColoredMeshDisplay
    : public rviz::MessageFilterDisplay<cgal_msgs::ColoredMesh> {
  Q_OBJECT
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ColoredMeshDisplay() {}
  virtual ~ColoredMeshDisplay();

  void initProperties();

 private Q_SLOTS:
  // Property Callbacks
  void backfaceCullingPropertyChanged();
  void appearencePropertyChanged();

 protected:
  virtual void onInitialize();
  virtual void reset();

 private:
  void processMessage(const cgal_msgs::ColoredMesh::ConstPtr &msg);

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

#endif  // COLORED_MESH_DISPLAY_VISUALIZATIONS_H
