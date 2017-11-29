#include "Planet.h"

#include <osg/Texture2D>
#include <osgDB/ReadFile>

#include "../config.h"
#include "../osg/OsgEigenConversions.h"
#include "../osg/ImageManager.h"
#include "../osg/ModelManager.h"

using namespace pbs17;


/**
 * \brief Constructor of Planet.
 *
 * \param size
 *      Size of the planet.
 */
Planet::Planet(double size)
    : SpaceObject("", 0), _size(size) {
}

Planet::Planet(json j) : SpaceObject(j){
    std::cout << "json=" << j << std::endl;
    _size = j["size"].get<double>();
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
 * \brief Constructor of Planet.
 *
 * \param size
 *      Size of the planet.
 * \param textureName
 *      Relative location to the texture-file. (Relative from the data-directory in the source).
 */
Planet::Planet(double size, std::string textureName)
    : SpaceObject("", textureName), _size(size) {
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
	_scaling = _size;

	// Load the model
	std::string modelPath = DATA_PATH + "/sphere.obj";
	osg::ref_ptr<osg::Node> modelFile = ModelManager::Instance()->loadModel(modelPath, ratio, _size);

	if (_textureName != "") {
		std::string texturePath = DATA_PATH + "/texture/" + _textureName;
        std::cout << texturePath << std::endl;
		osg::ref_ptr<osg::Texture2D> myTex = ImageManager::Instance()->loadTexture(texturePath);
		modelFile->getOrCreateStateSet()->setTextureAttributeAndModes(0, myTex.get());
	}

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
