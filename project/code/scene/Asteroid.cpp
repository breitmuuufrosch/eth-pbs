#include "Asteroid.h"

#include <osgDB/ReadFile>
#include <osg/TexGen>

#include "../config.h"
#include "../OsgEigenVector.h"
#include "../osg/ModelManager.h"

using namespace pbs17;


/**
* \brief Constructor of SpaceObject.
*
* \param filename
*      Relative location to the object-file. (Relative from the data-directory in the source).
* \param center
*      Center of the global-rotation.
*/
Asteroid::Asteroid(std::string filename, Eigen::Vector3d center)
	: SpaceObject(filename, center) {
}


Asteroid::~Asteroid() {}


/**
* \brief Initialize the space-object for OSG.
*
* \param position
*      Initial position of the object.
* \param scaling
*      Scaling of the model. (1.0 => not scaled, < 1.0 => smaller, > 1.0 => larger)
*/
void Asteroid::initOsg(Eigen::Vector3d position, double scaling) {
	// Set the position to the space-object
	_position = position;

	// Load the model
	std::string modelPath = DATA_PATH + "/" + _filename;
	osg::ref_ptr<osg::Node> modelFile = ModelManager::Instance()->loadModel(modelPath, 1.0, scaling);

	// First transformation-node to handle locale-rotations easier
	_rotation = new osg::MatrixTransform;
	_rotation->addChild(modelFile);

	// Second transformation-node for global rotations and translations
	_model = new osg::MatrixTransform;
	_model->setMatrix(osg::Matrix::translate(toOsg(position)));
	_model->addChild(_rotation);
}