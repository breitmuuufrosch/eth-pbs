#include "SpaceShip.h"

#include <osgDB/ReadFile>
#include <osg/Switch>

#include <Eigen/Geometry>

#include "../config.h"
#include "../osg/ModelManager.h"
#include "../osg/OsgEigenConversions.h"
#include "../osg/JsonEigenConversions.h"
#include "../osg/visitors/ConvexHullVisitor.h"

using namespace pbs17;


/**
* \brief Constructor of SpaceShip.
*/
SpaceShip::SpaceShip()
	: SpaceObject("starfighter.obj", 0) {}


/**
 * \brief Constructor of SpaceShip with JSON-configuration.
 *
 * \param j
 *      JSON-configuration for the space-ship.
 */
SpaceShip::SpaceShip(json j) :
	SpaceObject(j) {

	Eigen::Vector3d pos = fromJson(j["position"]);
	initOsg(pos, j["ratio"].get<double>(), j["scaling"].get<double>());

	Eigen::Vector3d linearVelocity = fromJson(j["linearVelocity"]);
	Eigen::Vector3d angularVelocity = fromJson(j["angularVelocity"]);
	Eigen::Vector3d force = fromJson(j["force"]);
	Eigen::Vector3d torque = fromJson(j["torque"]);

	initPhysics(j["mass"].get<double>(), linearVelocity, angularVelocity, force, torque);
}


/**
 * \brief Destructor of Asteroid.
 */
SpaceShip::~SpaceShip() {}


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
void SpaceShip::initOsg(Eigen::Vector3d position, double ratio, double scaling) {
	// Set the position to the space-object
	_position = position;
	_scaling = scaling;

	// Load the model
	std::string modelPath = DATA_PATH + "/" + _filename;
	_modelFile = ModelManager::Instance()->loadModel(modelPath, ratio, scaling);

	// Compute convex hull
	osg::Matrix scalingMatrix = osg::Matrix::scale(_scaling, _scaling, _scaling);
	ConvexHullVisitor convexHull(scalingMatrix);
	_modelFile->accept(convexHull);
	_convexHull = convexHull.getConvexHull();

	osg::Geode* geodeConvexHull = new osg::Geode;
	geodeConvexHull->addDrawable(_convexHull->getOsgModel());

	// Switch to decide if the convex hull or the model has to be rendered.
	_convexRenderSwitch = new osg::Switch;
	_convexRenderSwitch->addChild(_modelFile, true);
	_convexRenderSwitch->addChild(geodeConvexHull, false);

	// First transformation-node to handle locale-rotations easier
	_rotation = new osg::MatrixTransform;
	_rotation->addChild(_convexRenderSwitch);

	// Second transformation-node for global rotations and translations
	_translation = new osg::MatrixTransform;
	_translation->setMatrix(osg::Matrix::translate(toOsg(position)));
	_translation->addChild(_rotation);

	calculateAABB();

	_modelRoot = new osg::Switch;
	_modelRoot->addChild(_translation, true);
	_modelRoot->addChild(_aabbRendering, true);

	initTexturing();
}

/**
 * \brief Initialize the space-object for physics.
 *
 * \param mass
 *      Mass: unit = kg
 * \param linearVelocity
 *      Linear velocity: unit = m/s
 * \param angularVelocity
 *      Angular velocity: unit = rad/s
 * \param force
 *      Global force: unit = vector with norm equals to N
 * \param torque
 *      Global torque: unit = vector with norm equals to N*m (newton metre)
 */
void SpaceShip::initPhysics(double mass, Eigen::Vector3d linearVelocity, Eigen::Vector3d angularVelocity, Eigen::Vector3d force, Eigen::Vector3d torque) {
	SpaceObject::initPhysics(mass, linearVelocity, angularVelocity, force, torque);

	_momentOfInertia = Eigen::Matrix3d();
	_momentOfInertia.setIdentity();
}

void SpaceShip::turnUp() {
	// Todo
}

void SpaceShip::turnDown() {
	// Todo
}

void SpaceShip::turnLeft() {
	// Todo
}

void SpaceShip::turnRight() {
	// Todo
}

void SpaceShip::accelerate() {
	// Todo
}

void SpaceShip::decelerate() {
	// Todo
}
