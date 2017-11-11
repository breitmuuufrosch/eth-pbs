#include "Asteroid.h"

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
Asteroid::Asteroid(std::string filename, osg::Vec3 translate, osg::Vec3 center, double scaling)
	: SpaceObject(filename, center, scaling) {
	init(translate);
}


Asteroid::~Asteroid() {}


/**
* \brief Initialize the space-object for OSG.
*
* \param translate
*      Initial translation of the object.
*/
void Asteroid::init(osg::Vec3 translate) {
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
		modelFile = SpaceObject::scaleNode(modelFile, _scaling);
	}

	// First transformation-node to handle locale-rotations easier
	_rotation = new osg::MatrixTransform;
	_rotation->addChild(modelFile);

	// Second transformation-node for global rotations and translations
	_model = new osg::MatrixTransform;
	_model->setMatrix(osg::Matrix::translate(translate));
	_model->addChild(_rotation);
}