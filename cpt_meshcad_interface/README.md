# cpt_meshcad_interface

This package is based on hilti_slam/cad_interface. It offers the basic tools for alignment of a CAD model with the current scan. All other functionality was removed. Furthermore, a second Interactive Marker is used to get facet IDs. Compared to cad_interface, this package loads a mesh model from an .off file and generates a cgal message of the transformed model.

Please see cpt_selective_icp for instructions on how to use it.

Hint: 

- Alignment of marker through the mesh is not possible. Deactivate model visualization for easier access to marker.