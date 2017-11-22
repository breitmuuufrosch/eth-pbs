#include "Asteroid.h"

#include <osgDB/ReadFile>

#include "../config.h"
#include "../osg/ModelManager.h"
#include "../osg/OsgEigenConversions.h"

using namespace pbs17;


/**
* \brief Constructor of Asteroid.
*
* \param filename
*      Relative location to the object-file. (Relative from the data-directory in the source).
*/
Asteroid::Asteroid()
    : SpaceObject("A2.obj", 0) {

}

Asteroid::Asteroid(json j) :
    SpaceObject(j){

    Eigen::Vector3d pos(
        j["position"]["x"].get<double>(),
        j["position"]["y"].get<double>(),
        j["position"]["z"].get<double>());
    initOsg(pos, j["ratio"].get<double>(), j["scaling"].get<double>());

    Eigen::Vector3d linearVelocity(
        j["linearVelocity"]["x"].get<double>(),
        j["linearVelocity"]["y"].get<double>(),
        j["linearVelocity"]["z"].get<double>());
    Eigen::Vector3d angularVelocity(
        j["angularVelocity"]["x"].get<double>(),
        j["angularVelocity"]["y"].get<double>(),
        j["angularVelocity"]["z"].get<double>());
    Eigen::Vector3d force(
        j["force"]["x"].get<double>(),
        j["force"]["y"].get<double>(),
        j["force"]["z"].get<double>());
    Eigen::Vector3d torque(
        j["torque"]["x"].get<double>(),
        j["torque"]["y"].get<double>(),
        j["torque"]["z"].get<double>());


    initPhysics(j["mass"].get<double>(),linearVelocity,angularVelocity,force, torque);
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
	_scaling = scaling;

	// Load the model
	std::string modelPath = DATA_PATH + "/" + _filename;
	osg::ref_ptr<osg::Node> modelFile = ModelManager::Instance()->loadModel(modelPath, ratio, scaling);

	// First transformation-node to handle locale-rotations easier
	_rotation = new osg::MatrixTransform;
	_rotation->addChild(modelFile);

	// Second transformation-node for global rotations and translations
	_translation = new osg::MatrixTransform;
	_translation->setMatrix(osg::Matrix::translate(toOsg(position)));
	_translation->addChild(_rotation);

	calculateAABB();

	_model = new osg::Group;
	_model->addChild(_aabbRendering);
	_model->addChild(_translation);
}


void Asteroid::setOrientation(Eigen::Vector3d o) {
	SpaceObject::setOrientation(o);

	calculateAABB();
}
