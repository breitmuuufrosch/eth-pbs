/**
* \brief Functionality for managing loaded models to prevent loading multiple times the same model.
* The code is copied from http://www.vis-sim.com/osg/code/osgcode_bbox1.htm and adapted to our use.
*
* \Author: Alexander Lelidis (14-907-562), Andreas Emch (08-631-384), Uroš Tešić (17-950-346)
* \Date:   2017-11-11
*/

#include "VertexListVisitor.h"

#include <osg/MatrixTransform>
#include <osg/Geometry>
#include <iostream>

using namespace pbs17;


/**
* \brief Calculate the bounding-box for the type osg::Geode.
*
* \param geode
*      Current geode-child.
*/
void VertexListVisitor::apply(osg::Geode& geode) {

	// update bounding box for each drawable
	for (unsigned int i = 0; i < geode.getNumDrawables(); ++i) {
		osg::Geometry *curGeom = geode.getDrawable(i)->asGeometry();

		// Only process if the drawable is geometry
		if (curGeom) {
			m_vertices = dynamic_cast<osg::Vec3Array*>(curGeom->getVertexArray());

			
		} else {
			std::cout << "Consider using OBJ-files instead of shapes.";
			//osg::ShapeDrawable* drawable = dynamic_cast<osg::ShapeDrawable*>(geode.getDrawable(i));
			//curGeom = drawable->asGeometry();
			//bbox.expandBy(geode.getDrawable(i)->getBound());
			//bbox.expandBy(geode.getDrawable(i)->getBoundingBox());
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
void VertexListVisitor::apply(osg::MatrixTransform& node) {

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
void VertexListVisitor::apply(osg::Billboard& node) {
	traverse(node);
}
