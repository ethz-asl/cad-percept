//
// Created by mpantic on 20.02.19.
//

#ifndef CGAL_DEFINITIONS_CGAL_TYPEDEFS_H
#define CGAL_DEFINITIONS_CGAL_TYPEDEFS_H

#include <CGAL/Simple_cartesian.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_items_with_id_3.h>
#include <CGAL/Side_of_triangle_mesh.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/property_map.h>

// tree typdefs
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_tree.h>

// Domain typdefs
#include <CGAL/Labeled_mesh_domain_3.h>
#include <CGAL/Mesh_complex_3_in_triangulation_3.h>

namespace cad_percept {
namespace cgal {

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::FT FT;
typedef Kernel::Point_3 Point;
typedef Kernel::Point_2 Point_2;
typedef Kernel::Plane_3 Plane;
typedef Kernel::Ray_3 Ray;
typedef Kernel::Segment_3 Segment;
typedef Kernel::Segment_2 Segment_2;
typedef Kernel::Sphere_3 Sphere;
typedef Kernel::Triangle_3 Triangle;
typedef Kernel::Vector_3 Vector;

// for meshing
typedef CGAL::Labeled_mesh_domain_3<Kernel, int, int> Mesh_domain;

typedef CGAL::Aff_transformation_3<Kernel> Transformation;

typedef CGAL::Polyhedron_3<Kernel, CGAL::Polyhedron_items_with_id_3> Polyhedron;
typedef std::shared_ptr<Polyhedron> PolyhedronPtr;

// datastructures
typedef Polyhedron::HalfedgeDS HalfedgeDS;

typedef boost::graph_traits<Polyhedron>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<Polyhedron>::halfedge_descriptor halfedge_descriptor;
typedef boost::graph_traits<Polyhedron>::face_descriptor face_descriptor;

typedef boost::graph_traits<Polyhedron>::vertex_iterator vertex_iterator;
typedef boost::graph_traits<Polyhedron>::face_iterator face_iterator;

typedef CGAL::Mesh_complex_3_in_triangulation_3<Polyhedron, Polyhedron::Vertex_handle,
                                                Polyhedron::Face_handle>
    C3T3;

// Tree structures
// AABB tree
typedef CGAL::AABB_face_graph_triangle_primitive<Polyhedron>
    PolyhedronPrimitive;                                                      // cadify: Primitive
typedef CGAL::AABB_traits<Kernel, PolyhedronPrimitive> PolyhedronAABBTraits;  // cadify: Traits
typedef CGAL::AABB_tree<PolyhedronAABBTraits> PolyhedronAABBTree;             // cadify: Tree
typedef CGAL::Side_of_triangle_mesh<Polyhedron, Kernel> PolyhedronSideOf;

typedef boost::optional<PolyhedronAABBTree::Intersection_and_primitive_id<Segment>::Type>
    PolyhedronSegmentIntersection;
typedef boost::optional<PolyhedronAABBTree::Intersection_and_primitive_id<Plane>::Type>
    PolyhedronPlaneIntersection;
typedef boost::optional<PolyhedronAABBTree::Intersection_and_primitive_id<Ray>::Type>
    PolyhedronRayIntersection;  // cadify: Ray_intersection
typedef PolyhedronAABBTree::Primitive_id PolyhedronPrimitiveId;
typedef std::pair<Point, PolyhedronPrimitive::Id> PointAndPrimitiveId;
}  // namespace cgal
}  // namespace cad_percept
#endif  // CGAL_DEFINITIONS_CGAL_TYPEDEFS_H
