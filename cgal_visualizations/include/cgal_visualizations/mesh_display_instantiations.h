#ifndef CGAL_VISUALIZATIONS_MESH_DISPLAY_INSTANTIATIONS_H_
#define CGAL_VISUALIZATIONS_MESH_DISPLAY_INSTANTIATIONS_H_
#include <cgal_visualizations/mesh_display.h>
namespace cad_percept {
namespace visualizations {
/*
 * That's where the magic happens -
 *  For each message type, MeshDisplay is inherited once with the Q_OBJECT macro.
 *  The Q_SLOTS qualifier adds the properly named slots and these are redirected into its baseclass
 *  implementation. This does not make sense C++-wise, but the QT compiler needs this to generate
 *  the appropriate SLOT names.
 *  It's three times the same code, but way less duplication than copying the classes completely.
 */

/*
 * For TriangleMeshStamped
 */
class TriangleMeshDisplay : public MeshDisplay<cgal_msgs::TriangleMeshStamped> {
  Q_OBJECT
 protected Q_SLOTS:
  inline void backfaceCullingPropertyChanged() final {
    MeshDisplay::backfaceCullingPropertyChanged();
  }
  inline void appearencePropertyChanged() final { MeshDisplay::appearencePropertyChanged(); }
};

/*
 * For ColoredMesh
 */
class ColoredMeshDisplay : public MeshDisplay<cgal_msgs::ColoredMesh> {
  Q_OBJECT
 protected Q_SLOTS:
  inline void backfaceCullingPropertyChanged() final {
    MeshDisplay::backfaceCullingPropertyChanged();
  }
  inline void appearencePropertyChanged() final { MeshDisplay::appearencePropertyChanged(); }
};

/*
 * For ProbabilisticMesh
 */
class ProbabilisticMeshDisplay : public MeshDisplay<cgal_msgs::ProbabilisticMesh> {
  Q_OBJECT
 protected Q_SLOTS:
  inline void backfaceCullingPropertyChanged() final {
    MeshDisplay::backfaceCullingPropertyChanged();
  }
  inline void appearencePropertyChanged() final { MeshDisplay::appearencePropertyChanged(); }
};

}  // namespace visualizations
}  // namespace cad_percept

#endif  // CGAL_VISUALIZATIONS_MESH_DISPLAY_INSTANTIATIONS_H_
