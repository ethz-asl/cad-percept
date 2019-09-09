#ifndef CGAL_VISUALIZATIONS_Q_MESH_DISPLAY_H_
#define CGAL_VISUALIZATIONS_Q_MESH_DISPLAY_H_

#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <Eigen/Dense>

/*
 * Abstract class that handles QT specific interfaces
 * Introduced to break problem with Q_OBJECTS not being allowed to be templated.
 * (Mainly Q_SLOTS and Q_OBJECT).
 */
class QMeshDisplay {
  Q_OBJECT

 protected Q_SLOTS:
  /*
   * Pure virtual Q_SLOTS, that are defined in the subclass.
   */
  virtual void backfaceCullingPropertyChanged() = 0;
  virtual void appearencePropertyChanged() = 0;
};

#endif  // CGAL_VISUALIZATIONS_Q_MESH_DISPLAY_H_
