#include "SpaceShip.h"

#include <osgDB/ReadFile>
#include <osg/Switch>

#include <Eigen/Geometry>

#include "../config.h"
#include "../osg/ModelManager.h"
#include "../osg/OsgEigenConversions.h"
#include "../osg/JsonEigenConversions.h"
#include "../osg/visitors/ConvexHullVisitor.h"
#include "../osg/particles/SmokeParticleSystem.h"

using namespace pbs17;


/**
* \brief Constructor of SpaceShip.
*/
SpaceShip::SpaceShip()
    : SpaceObject("starfighter.obj", 0) {
    Eigen::Vector3d pos = Eigen::Vector3d(0.0, 0.0, 0.0);
    initOsg(pos, 1.0, 1.0);

    Eigen::Vector3d linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
    Eigen::Vector3d angularVelocity = Eigen::Vector3d(0.0, 0.0, 0.0);
    Eigen::Vector3d force = Eigen::Vector3d(0.0, 0.0, 0.0);;
    Eigen::Vector3d torque = Eigen::Vector3d(0.0, 0.0, 0.0);;

    initPhysics(1.0, linearVelocity, angularVelocity, force, torque);

}


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

	// Transformation-node for position and rotation updates.
	_transformation = new osg::MatrixTransform;
	_transformation->setMatrix(osg::Matrix::translate(toOsg(position)));
	_transformation->addChild(_convexRenderSwitch);

	calculateAABB();

	// Particle system
	osg::ref_ptr<SmokeParticleSystem> smoke = new SmokeParticleSystem(_transformation, _transformation.get());

	_modelRoot = new osg::Switch;
	_modelRoot->addChild(_transformation, true);
	_modelRoot->addChild(_aabbRendering, true);
	_modelRoot->addChild(smoke, true);

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
    Eigen::Vector3d p = getPosition();
    osg::Quat q;
    double sinQuat = sin(_rotationAngle / 2);
    double cosQuat = cos(_rotationAngle / 2);

    q.set(sinQuat * 0.0,
          sinQuat * 1.0,
          sinQuat * 0.0,
          cosQuat);
    osg::Quat newQ = q * getOrientation();

    updatePositionOrientation(p, newQ);
    updateDirectionOrientation(getLinearVelocity(), newQ);
}

void SpaceShip::turnDown() {
    Eigen::Vector3d p = getPosition();
    osg::Quat q;
    double sinQuat = sin(_rotationAngle / 2);
    double cosQuat = cos(_rotationAngle / 2);

    q.set(sinQuat * 0.0,
          sinQuat * -1.0,
          sinQuat * 0.0,
          cosQuat);
    osg::Quat newQ = q * getOrientation();

    updatePositionOrientation(p, newQ);
    updateDirectionOrientation(getLinearVelocity(), newQ);
}

void SpaceShip::turnLeft() {
    Eigen::Vector3d p = getPosition();
    osg::Quat q;
    double sinQuat = sin(_rotationAngle / 2);
    double cosQuat = cos(_rotationAngle / 2);

    q.set(sinQuat * 1.0,
          sinQuat * 0.0,
          sinQuat * 0.0,
          cosQuat);
    osg::Quat newQ = q * getOrientation();

    updatePositionOrientation(p, newQ);
    updateDirectionOrientation(getLinearVelocity(), newQ);
}

void SpaceShip::turnRight() {
    Eigen::Vector3d p = getPosition();
    osg::Quat q;
    double sinQuat = sin(_rotationAngle / 2);
    double cosQuat = cos(_rotationAngle / 2);

    q.set(sinQuat * -1.0,
          sinQuat * 0.0,
          sinQuat * 0.0,
          cosQuat);
    osg::Quat newQ = q * getOrientation();

    updatePositionOrientation(p, newQ);
    updateDirectionOrientation(getLinearVelocity(), newQ);
}

void SpaceShip::accelerate() {
    Eigen::Vector3d v = getLinearVelocity() * _acceleration;
    if(v.norm() < 0.0001) {
        v = Eigen::Vector3d (1.0, 0.0, 0.0);
    }

    setLinearVelocity(v);
}

void SpaceShip::decelerate() {
    Eigen::Vector3d v = getLinearVelocity() * _decelerate;
    setLinearVelocity(v);
}
