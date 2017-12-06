#include "Asteroid.h"

#include <osgDB/ReadFile>
#include <osg/Switch>

#include "../config.h"
#include "../osg/ModelManager.h"
#include "../osg/OsgEigenConversions.h"
#include "../osg/JsonEigenConversions.h"
#include "../osg/visitors/ConvexHullVisitor.h"
#include "../osg/ImageManager.h"
#include "../osg/visitors/ComputeTangentVisitor.h"
#include "../osg/shaders.h"

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
	_convexRenderSwitch->addChild(_modelFile, true);
	_convexRenderSwitch->addChild(geodeConvexHull, false);

	// Apply bumpmap-shaders
	ComputeTangentVisitor ctv;
	ctv.setTraversalMode(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN);
	_modelFile->accept(ctv);

	osg::ref_ptr<osg::Program> program = new osg::Program;
	program->addShader(new osg::Shader(osg::Shader::VERTEX, vertBumpMap));
	program->addShader(new osg::Shader(osg::Shader::FRAGMENT, fragBumpMap));
	program->addBindAttribLocation("tangent", 6);
	program->addBindAttribLocation("binormal", 7);

	std::string bumpmapPath = DATA_PATH + "/texture/whitemetal_normal.jpg";
	std::cout << bumpmapPath << std::endl;
	osg::ref_ptr<osg::Texture2D> normalTex = ImageManager::Instance()->loadTexture(bumpmapPath);

	// Load the texture
	if (_textureName != "") {
		std::string texturePath = DATA_PATH + "/texture/whitemetal_diffuse.jpg"; // +_textureName;
		std::cout << texturePath << std::endl;
		osg::ref_ptr<osg::Texture2D> colorTex = ImageManager::Instance()->loadTexture(texturePath);

		osg::ref_ptr<osg::StateSet> stateset = _modelFile->getOrCreateStateSet();
		//stateset->setTextureAttributeAndModes(0, myTex.get());

		stateset->addUniform(new osg::Uniform("colorTex", 0));
		stateset->addUniform(new osg::Uniform("normalTex", 1));
		stateset->setAttributeAndModes(program.get());

		osg::StateAttribute::GLModeValue value = osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE;
		stateset->setTextureAttributeAndModes(0, colorTex.get(), value);
		stateset->setTextureAttributeAndModes(1, normalTex.get(), value);
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
void Asteroid::initPhysics(double mass, Eigen::Vector3d linearVelocity, Eigen::Vector3d angularVelocity, Eigen::Vector3d force, Eigen::Vector3d torque) {
	SpaceObject::initPhysics(mass, linearVelocity, angularVelocity, force, torque);

	_momentOfInertia = Eigen::Matrix3d();
	_momentOfInertia.setIdentity();
}