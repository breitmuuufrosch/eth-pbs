#pragma once

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_items_with_id_3.h>
#include <CGAL/Nef_polyhedron_3.h>

#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/Side_of_triangle_mesh.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel              K;
typedef CGAL::Polyhedron_3<K, CGAL::Polyhedron_items_with_id_3>        Polyhedron_3;
typedef CGAL::Polyhedron_3<K>                                          Polyhedron_No_Id_3;
typedef CGAL::Nef_polyhedron_3<K>                                      Nef_Polyhedron_3;
typedef K::Segment_3                                                   Segment_3;
typedef K::Point_3                                                     Point_3;

typedef Polyhedron_3::Vertex_iterator                                  Vertex_iterator;
typedef Polyhedron_3::Facet_iterator                                   Facet_iterator;
typedef Polyhedron_3::Halfedge_around_facet_circulator                 Halfedge_around_facet_circulator;
typedef Polyhedron_3::HalfedgeDS                                       HalfedgeDS;
typedef Nef_Polyhedron_3::Vertex_iterator                              Nef_Vertex_iterator;
typedef Nef_Polyhedron_3::Vector_3                                     Vector_3;
typedef Nef_Polyhedron_3::Aff_transformation_3                         Aff_transformation_3;

typedef CGAL::AABB_face_graph_triangle_primitive<Polyhedron_No_Id_3>   Primitive;
typedef CGAL::AABB_traits<K, Primitive>                                Traits;
typedef CGAL::AABB_tree<Traits>                                        Tree;
typedef CGAL::Side_of_triangle_mesh<Polyhedron_No_Id_3, K>             Point_inside;
