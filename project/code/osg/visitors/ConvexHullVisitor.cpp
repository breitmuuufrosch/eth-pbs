/**
* \brief Functionality for managing loaded models to prevent loading multiple times the same model.
* The code is copied from http://www.vis-sim.com/osg/code/osgcode_bbox1.htm and adapted to our use.
*
* \Author: Alexander Lelidis (14-907-562), Andreas Emch (08-631-384), Uroš Tešić (17-950-346)
* \Date:   2017-11-23
*/

#include "ConvexHullVisitor.h"

#include <iostream>
#include <vector>

#include <osg/MatrixTransform>
#include <osg/Geometry>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/point_generators_3.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_items_with_id_3.h>
#include <CGAL/convex_hull_3.h>

using namespace pbs17;

typedef CGAL::Exact_predicates_inexact_constructions_kernel  K;
//typedef CGAL::
typedef CGAL::Polyhedron_3<K, CGAL::Polyhedron_items_with_id_3>                     Polyhedron_3;
typedef K::Segment_3                              Segment_3;
// define point creator
typedef K::Point_3                                Point_3;
typedef Polyhedron_3::Vertex_iterator             Vertex_iterator;
typedef Polyhedron_3::Facet_iterator              Facet_iterator;
typedef Polyhedron_3::Halfedge_around_facet_circulator Halfedge_around_facet_circulator;


/**
* \brief Calculate the bounding-box for the type osg::Geode.
*
* \param geode
*      Current geode-child.
*/
void ConvexHullVisitor::apply(osg::Geode& geode) {

	// update bounding box for each drawable
	for (unsigned int i = 0; i < geode.getNumDrawables(); ++i) {
		osg::Geometry *curGeom = geode.getDrawable(i)->asGeometry();

		// Only process if the drawable is geometry
		if (curGeom) {
			osg::Vec3Array* vertices = dynamic_cast<osg::Vec3Array*>(curGeom->getVertexArray());

			if (vertices) {
				// Store all vertices of the subtree
				for (unsigned int j = 0; j < vertices->size(); ++j) {
					osg::Vec3 vertex_at_j = (*vertices)[j] * _transformMatrix;
					_vertices->push_back(vertex_at_j);
				}
			}
		} else {
			std::cout << "Consider using OBJ-files instead of shapes.";
		}
	}

	// continue traversing through the graph
	traverse(geode);
}


/**
* \brief Calculate the bounding-box for the type osg::MatrixTransform.
*
* \param node
*      Current matrix-transform-child.
*/
void ConvexHullVisitor::apply(osg::MatrixTransform& node) {
	_transformMatrix *= node.getMatrix();

	// continue traversing through the graph
	traverse(node);
}


/**
* \brief Calculate the bounding-box for the type osg::Billboard.
* important to handle billboard so that its size will not affect the geode size continue traversing the graph
*
* \param node
*      Current billboard-child.
*/
void ConvexHullVisitor::apply(osg::Billboard& node) {
	traverse(node);
}


osg::ref_ptr<osg::Geometry> ConvexHullVisitor::getConvexHull() {
	if (!_isCalculated) {
		std::vector<Point_3> points(_vertices->size());

		// Transform the osg-vertices into cgal-points
		for (unsigned int i = 0; i < _vertices->size(); ++i) {
			osg::Vec3 vertex_at_j = (*_vertices)[i];
			points[i] = Point_3(vertex_at_j[0], vertex_at_j[1], vertex_at_j[2]);
		}

		// Define polyhedron to hold convex hull and compute convex hull of non-collinear points
		Polyhedron_3 poly;
		CGAL::convex_hull_3(points.begin(), points.end(), poly);

		// Vectors to store the vertices and faces of the convex-hull
		osg::ref_ptr<osg::Vec3Array> convexVertices = new osg::Vec3Array;
		osg::ref_ptr < osg::DrawElementsUInt> convexFaces = new osg::DrawElementsUInt(GL_TRIANGLES);

		// Set the index of each vertex and add it to the convex-hull-vertices (osg)
		unsigned int index = 0;
		for (Vertex_iterator v = poly.vertices_begin(); v != poly.vertices_end(); ++v, ++index) {
			v->id() = index;
			convexVertices->push_back(osg::Vec3(v->point()[0], v->point()[1], v->point()[2]));
		}

		// Store each face of the convex-hull
		for (Facet_iterator pFacet = poly.facets_begin(); pFacet != poly.facets_end(); ++pFacet) {
			Halfedge_around_facet_circulator pHalfedge = pFacet->facet_begin();
			do {
				convexFaces->push_back(pHalfedge->vertex()->id());
			} while (++pHalfedge != pFacet->facet_begin());
		}

		// Create the osg-entity which represents the convex-hull
		_convexHull = new osg::Geometry;
		_convexHull->setVertexArray(convexVertices);
		_convexHull->addPrimitiveSet(convexFaces);
		_isCalculated = true;
	}

	return _convexHull;
}
