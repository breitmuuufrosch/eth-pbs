/**
 * \brief Color visitor to colorize a node.
 *
 * \Author: Alexander Lelidis (14-907-562), Andreas Emch (08-631-384), Uroš Tešić (17-950-346)
 * \Date:   2017-11-16
 */

#include "ColorVisitor.h"

#include <osg/Node>
#include <osg/Geode>
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osg/Geometry>
#include <osg/ShapeDrawable>

using namespace pbs17;


ColorVisitor::ColorVisitor(const osg::Vec4 &color) : NodeVisitor(NodeVisitor::TRAVERSE_ALL_CHILDREN) {
	_color = color;
	_colorArrays = new osg::Vec4Array;
	_colorArrays->push_back(_color);
}


ColorVisitor::~ColorVisitor() {}


void ColorVisitor::apply(osg::Node &node) {
	traverse(node);
}


void ColorVisitor::apply(osg::Group &group) {
	traverse(group);
}


void ColorVisitor::apply(osg::MatrixTransform& matrixTransform) {
	traverse(matrixTransform);
}


void ColorVisitor::apply(osg::Geode &geode) {
	// We need to iterate through all the drawables check if the contain any geometry that we will need to process
	unsigned int numGeoms = geode.getNumDrawables();

	for (unsigned int geodeIdx = 0; geodeIdx < numGeoms; geodeIdx++) {
		// Use 'asGeometry' as its supposed to be faster than a dynamic_cast every little saving counts
		osg::Geometry *curGeom = geode.getDrawable(geodeIdx)->asGeometry();

		// Only process if the drawable is geometry
		if (curGeom) {
			curGeom->setColorArray(_colorArrays.get());
			curGeom->setColorBinding(osg::Geometry::BIND_OVERALL);
			continue;
		}

		osg::ShapeDrawable *curShape = dynamic_cast<osg::ShapeDrawable*>(geode.getDrawable(geodeIdx));

		if (curShape) {
			curShape->setColor(_color);
			continue;
		}
	}
}