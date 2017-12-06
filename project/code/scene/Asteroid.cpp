#include "Asteroid.h"
#include "../osg/visitors/VertexListVisitor.h"

#include <osgDB/ReadFile>
#include <osg/Switch>

#include "../config.h"
#include "../osg/ModelManager.h"
#include "../osg/OsgEigenConversions.h"
#include "../osg/JsonEigenConversions.h"
#include "../osg/visitors/ConvexHullVisitor.h"
#include "../osg/ImageManager.h"

using namespace pbs17;


/**
* \brief Constructor of Asteroid.
*/
Asteroid::Asteroid()
    : SpaceObject("A2.obj", 0) {
}


/**
 * \brief Constructor of Asteroid with JSON-configuration.
 * 
 * \param j
 *      JSON-configuration for the asteroid.
 */
Asteroid::Asteroid(json j) :
    SpaceObject(j) {

    Eigen::Vector3d pos = fromJson(j["position"]);
    initOsg(pos, j["ratio"].get<double>(), j["scaling"].get<double>());

	Eigen::Vector3d linearVelocity = fromJson(j["linearVelocity"]);
    Eigen::Vector3d angularVelocity = fromJson(j["angularVelocity"]);
    Eigen::Vector3d force = fromJson(j["force"]);
    Eigen::Vector3d torque = fromJson(j["torque"]);

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
	_convexRenderSwitch->addChild(_modelFile, false);
	_convexRenderSwitch->addChild(geodeConvexHull, true);

	// Load the texture
	if (_textureName != "") {
		std::string texturePath = DATA_PATH + "/texture/" + _textureName;
		std::cout << texturePath << std::endl;
		osg::ref_ptr<osg::Texture2D> myTex = ImageManager::Instance()->loadTexture(texturePath);
		_convexRenderSwitch->getOrCreateStateSet()->setTextureAttributeAndModes(0, myTex.get());
	}
	
	const osg::Vec3Array* vertices = dynamic_cast<const osg::Vec3Array*>(vListVisitor.getVertices());

	double minX = DBL_MAX, maxX = -DBL_MAX, minY = DBL_MAX, maxY = -DBL_MAX, minZ = DBL_MAX, maxZ = -DBL_MAX;

	for (int i = 0; i < vertices->size(); i++) {
		if (vertices->at(i).x() < minX) {
			minX = vertices->at(i).x();
		}

		if (vertices->at(i).x() > maxX) {
			maxX = vertices->at(i).x();
		}

		if (vertices->at(i).y() < minY) {
			minY = vertices->at(i).y();
		}

		if (vertices->at(i).y() > maxY) {
			maxY = vertices->at(i).y();
		}

		if (vertices->at(i).z() < minZ) {
			minZ = vertices->at(i).z();
		}

		if (vertices->at(i).z() > maxZ) {
			maxZ = vertices->at(i).z();
		}
	}
	double a = maxX - minX;
	double b = maxY - minY;
	double c = maxZ - minZ;
	double Ixx = 1. / 12.*(b*b + c*c) * scaling * scaling;
	double Iyy = 1. / 12.*(a*a + c*c) * scaling * scaling;
	double Izz = 1. / 12.*(a*a + b*b) * scaling * scaling;

	_momentOfInertia = Eigen::Matrix3d();
	_momentOfInertia.setIdentity();
	_momentOfInertia(0, 0) = Ixx;
	_momentOfInertia(1, 1) = Iyy;
	_momentOfInertia(2, 2) = Izz;

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
void Asteroid::initPhysics(double mass, Eigen::Vector3d linearVelocity, Eigen::Vector3d angularVelocity, Eigen::Vector3d force, Eigen::Vector3d torque) {
	SpaceObject::initPhysics(mass, linearVelocity, angularVelocity, force, torque);

	_momentOfInertia = mass * _momentOfInertia();
}