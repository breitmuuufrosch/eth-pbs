#include "Planet.h"

#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Texture2D>
#include <osgDB/ReadFile>

#include "../config.h"
#include "../osg/OsgEigenConversions.h"
#include "../osg/visitors/BoundingBoxVisitor.h"

using namespace pbs17;


/**
 * \brief Constructor of Planet.
 *
 * \param size
 *      Size of the planet.
 * \param center
 *      Center of the global-rotation.
 */
Planet::Planet(double size, Eigen::Vector3d center)
	: SpaceObject(""), _size(size) {
}

Planet::Planet(double size, Eigen::Vector3d center, std::string textureName)
    : SpaceObject("", textureName), _size(size) {
}

void Planet::initPhysics(double mass, Eigen::Vector3d linearVelocity, Eigen::Vector3d angularVelocity, Eigen::Vector3d force, Eigen::Vector3d torque) {
	SpaceObject::initPhysics(mass, linearVelocity, angularVelocity, force, torque);
	_momentOfInertia = Eigen::Matrix3d();
	_momentOfInertia.setIdentity();
	_momentOfInertia *= mass;
	_momentOfInertia *= 0.4 * getRadius() * getRadius();
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

	// Load the model
	osg::ref_ptr<osg::ShapeDrawable> model = new osg::ShapeDrawable;
	model->setShape(new osg::Sphere(osg::Vec3d(0.0f, 0.0f, 0.0f), _size));
	model->setColor(osg::Vec4d(1.0f, 1.0f, 1.0f, 1.0f));

    if(_textureName != "") {
        std::string texturePath = DATA_PATH + "/texture/" + _textureName;
        osg::ref_ptr<osg::Texture2D> myTex = new osg::Texture2D(osgDB::readImageFile(texturePath));
        model->getOrCreateStateSet()->setTextureAttributeAndModes(0, myTex.get());
    }

	osg::ref_ptr<osg::Geode> geometry = new osg::Geode;
	geometry->addDrawable(model);

	// First transformation-node to handle locale-rotations easier
	_rotation = new osg::MatrixTransform;
	_rotation->addChild(geometry);

	// Second transformation-node for global rotations and translations
	_model = new osg::MatrixTransform;
	_model->setMatrix(osg::Matrix::translate(toOsg(position)));
	_model->addChild(_rotation);

    CalculateBoundingBox bbox;
    _model->accept(bbox);
    _aabb = bbox.getBoundBox();
}
