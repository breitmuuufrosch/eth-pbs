#include "Planet.h"

#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/TexGen>
#include "../OsgEigenVector.h"

using namespace pbs17;


/**
 * \brief Constructor of SpaceObject.
 *
 * \param size
 *      Size of the planet.
 * \param center
 *      Center of the global-rotation.
 */
Planet::Planet(double size, Eigen::Vector3d center)
	: SpaceObject("", center), _size(size) {
}


Planet::~Planet() {}


/**
 * \brief Initialize the space-object for OSG.
 *
 * \param position
 *      Initial position of the object.
 * \param scaling
 *      Scaling of the model. (1.0 => not scaled, < 1.0 => smaller, > 1.0 => larger)
 */
void Planet::initOsg(Eigen::Vector3d position, double scaling) {
	// Set the position to the space-object
	_position = position;

	// Load the model
	osg::ref_ptr<osg::ShapeDrawable> model = new osg::ShapeDrawable;
	model->setShape(new osg::Sphere(osg::Vec3(0.0f, 0.0f, 0.0f), _size));
	model->setColor(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));

	osg::ref_ptr<osg::Geode> geometry = new osg::Geode;
	geometry->addDrawable(model);

	// First transformation-node to handle locale-rotations easier
	_rotation = new osg::MatrixTransform;
	_rotation->addChild(geometry);

	// Second transformation-node for global rotations and translations
	_model = new osg::MatrixTransform;
	_model->setMatrix(osg::Matrix::translate(toOsg(position)));
	_model->addChild(_rotation);
}