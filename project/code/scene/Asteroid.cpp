#include "Asteroid.h"

#include <osgDB/ReadFile>

#include "../config.h"
#include "../osg/ModelManager.h"
#include "../osg/OsgEigenConversions.h"
#include "../osg/visitors/BoundingBoxVisitor.h"

using namespace pbs17;


/**
* \brief Constructor of Asteroid.
*
* \param filename
*      Relative location to the object-file. (Relative from the data-directory in the source).
* \param center
*      Center of the global-rotation.
*/
Asteroid::Asteroid(std::string filename, Eigen::Vector3d center)
	: SpaceObject(filename) {
}


/**
 * \brief Destructor of Asteroid.
 */
Asteroid::~Asteroid() {}


/**
* \brief Initialize the space-object for OSG.
*
* \param position
*      Initial position of the object.
 * \param ratio
 *      Ratio of the simplifier. (Supported values: [0..1])
* \param scaling
*      Scaling of the model. (1.0 => not scaled, < 1.0 => smaller, > 1.0 => larger)
*/
void Asteroid::initOsg(Eigen::Vector3d position, double ratio, double scaling) {
	// Set the position to the space-object
	_position = position;

	// Load the model
	std::string modelPath = DATA_PATH + "/" + _filename;
	osg::ref_ptr<osg::Node> modelFile = ModelManager::Instance()->loadModel(modelPath, ratio, scaling);

	// First transformation-node to handle locale-rotations easier
	_rotation = new osg::MatrixTransform;
	_rotation->addChild(modelFile);

	// Second transformation-node for global rotations and translations
	_model = new osg::MatrixTransform;
	_model->setMatrix(osg::Matrix::translate(toOsg(position)));
	_model->addChild(_rotation);

	CalculateBoundingBox bbox;
	_model->accept(bbox);
    _aabb = bbox.getBoundBox();
}
