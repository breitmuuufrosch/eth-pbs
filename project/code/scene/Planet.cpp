#include "Planet.h"

#include <osg/Texture2D>
#include <osg/Switch>
#include <osgDB/ReadFile>

#include "../config.h"
#include "../osg/JsonEigenConversions.h"
#include "../osg/OsgEigenConversions.h"
#include "../osg/ImageManager.h"
#include "../osg/ModelManager.h"
#include "../osg/visitors/ConvexHullVisitor.h"

using namespace pbs17;


/**
 * \brief Constructor of Planet.
 *
 * \param size
 *      Size of the planet.
 */
Planet::Planet(double size)
    : SpaceObject("", 0), _radius(size) {
}


/**
 * \brief Constructor of Planet with JSON-configuration.
 *
 * \param j
 *      JSON-configuration for the planet.
 */
Planet::Planet(json j) : SpaceObject(j){
    std::cout << "json=" << j << std::endl;
    _radius = j["size"].get<double>();
	Eigen::Vector3d pos = fromJson(j["position"]);
    initOsg(pos, j["ratio"].get<double>(), j["scaling"].get<double>());

	Eigen::Vector3d linearVelocity = fromJson(j["linearVelocity"]);
	Eigen::Vector3d angularVelocity = fromJson(j["angularVelocity"]);
	Eigen::Vector3d force = fromJson(j["force"]);
	Eigen::Vector3d torque = fromJson(j["torque"]);

    initPhysics(j["mass"].get<double>(),linearVelocity,angularVelocity,force, torque);
}


/**
 * \brief Constructor of Planet.
 *
 * \param size
 *      Size of the planet.
 * \param textureName
 *      Relative location to the texture-file. (Relative from the data-directory in the source).
 */
Planet::Planet(double size, std::string textureName)
    : SpaceObject("", textureName), _radius(size) {
}


/**
 * \brief Destructor of Planet.
 */
Planet::~Planet() {}


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
void Planet::initOsg(Eigen::Vector3d position, double ratio, double scaling) {
	// Set the position to the space-object
	_position = position;
	_scaling = _radius;

	// Load the model
	std::string modelPath = DATA_PATH + "/sphere.obj";
	_modelFile = ModelManager::Instance()->loadModel(modelPath, ratio, _radius);

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

	// Load the texture
	if (_textureName != "") {
		std::string texturePath = DATA_PATH + "/texture/" + _textureName;
        std::cout << texturePath << std::endl;
		osg::ref_ptr<osg::Texture2D> myTex = ImageManager::Instance()->loadTexture(texturePath);
		_convexRenderSwitch->getOrCreateStateSet()->setTextureAttributeAndModes(0, myTex.get());
	}

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
void Planet::initPhysics(double mass, Eigen::Vector3d linearVelocity, Eigen::Vector3d angularVelocity, Eigen::Vector3d force, Eigen::Vector3d torque) {
	SpaceObject::initPhysics(mass, linearVelocity, angularVelocity, force, torque);

	_momentOfInertia = Eigen::Matrix3d();
	_momentOfInertia.setIdentity();
	_momentOfInertia *= mass;
	_momentOfInertia *= 0.4 * getRadius() * getRadius();
}
