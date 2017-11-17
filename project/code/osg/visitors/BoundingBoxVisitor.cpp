/**
* \brief Functionality for managing loaded models to prevent loading multiple times the same model.
* The code is copied from http://www.vis-sim.com/osg/code/osgcode_bbox1.htm and adapted to our use.
*
* \Author: Alexander Lelidis (14-907-562), Andreas Emch (08-631-384), Uroš Tešić (17-950-346)
* \Date:   2017-11-11
*/

#include "BoundingBoxVisitor.h"

#include <osg/MatrixTransform>
#include <osg/Geometry>

using namespace pbs17;


/**
* \brief Calculate the bounding-box for the type osg::Geode.
*
* \param geode
*      Current geode-child.
*/
void CalculateBoundingBox::apply(osg::Geode& geode) {
	osg::BoundingBox bbox;

	// update bounding box for each drawable
	for (unsigned int i = 0; i < geode.getNumDrawables(); ++i) {
		osg::Geometry *curGeom = geode.getDrawable(i)->asGeometry();

		// Only process if the drawable is geometry
		//bbox.expandBy(geode.getDrawable(i)->getBound());
		if (curGeom) {
			bbox.expandBy(curGeom->getBound());
		} else {
			bbox.expandBy(geode.getDrawable(i)->getBound());
		}
	}

	// transform corners by current matrix
	osg::BoundingBox bboxTrans;

	for (unsigned int i = 0; i < 8; ++i) {
		osg::Vec3 xvec = bbox.corner(i) * m_transformMatrix;
		bboxTrans.expandBy(xvec);
	}

	// update the overall bounding box size
	m_boundingBox.expandBy(bboxTrans);

	// continue traversing through the graph
	traverse(geode);
}


/**
* \brief Calculate the bounding-box for the type osg::MatrixTransform.
*
* \param node
*      Current matrix-transform-child.
*/
void CalculateBoundingBox::apply(osg::MatrixTransform& node) {
	m_transformMatrix *= node.getMatrix();

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
void CalculateBoundingBox::apply(osg::Billboard& node) {
	traverse(node);
}
