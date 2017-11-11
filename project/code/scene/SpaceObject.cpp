#include "SpaceObject.h"

#include <osgDB/ReadFile>
#include <osg/TexGen>

#include "../config.h"

using namespace pbs17;


/**
* \brief Constructor of SpaceObject.
*
* \param filename
*      Relative location to the object-file. (Relative from the data-directory in the source).
* \param translate
*      Initial translation of the object.
* \param center
*      Center of the global-rotation.
* \param scaling
*      Scaling of the model. (1.0 => not scaled, < 1.0 => smaller, > 1.0 => larger)
*/
SpaceObject::SpaceObject(std::string filename, osg::Vec3 translate, osg::Vec3 center, double scaling)
	: _filename(filename), _scaling(scaling), _center(center) {
	init(translate);
}


/**
* \brief Initialize the space-object for OSG.
*
* \param translate
*      Initial translation of the object.
*/
void SpaceObject::init(osg::Vec3 translate) {
	// Load the model
	std::string modelPath = DATA_PATH + "/" + _filename;
	std::cout << "Load model \"" << modelPath << "\"..." << std::endl;
	osg::ref_ptr<osg::Node> modelFile = osgDB::readNodeFile(modelPath);

	if (!modelFile) {
		std::cout << "File not found! Aborting..." << std::endl;
		exit(0);
	}

	// Scale the model if desired
	if (_scaling != 1.0) {
		modelFile = scaleNode(modelFile, _scaling);
	}

	// First transformation-node to handle locale-rotations easier
	_rotation = new osg::MatrixTransform;
	_rotation->addChild(modelFile);

	// Second transformation-node for global rotations and translations
	_model = new osg::MatrixTransform;
	_model->setMatrix(osg::Matrix::translate(translate));
	_model->addChild(_rotation);
}


/**
* \brief Set the local rotation of the model. (Around it's local axis).
*
* \param angle
*      Angle of the rotation.
* \param axis
*      Axis of the rotation.
*/
void SpaceObject::setLocalRotation(double angle, osg::Vec3 axis) const
{
	osg::Matrix rotationMatrix = osg::Matrix::rotate(angle, axis);
	_rotation->setMatrix(rotationMatrix * _rotation->getMatrix());
}


/**
* \brief Scales a OSG-node to the desired size (by ratio). All axis are scaled uniformly.
*
* \param node
*      Node which is scaled.
* \param scaling
*      Scaling-factor.
*
* \return Scaled OSG-node.
*/
osg::ref_ptr<osg::MatrixTransform> SpaceObject::scaleNode(osg::ref_ptr<osg::Node> node, double scaling) {
	std::cout << "Scaling node \"" << node->getName() << "\" with the ratio " << scaling << std::endl;

	osg::ref_ptr<osg::MatrixTransform> scaled = new osg::MatrixTransform;
	scaled->setMatrix(osg::Matrix::scale(scaling, scaling, scaling));
	scaled->addChild(node.get());

	return scaled;
}
