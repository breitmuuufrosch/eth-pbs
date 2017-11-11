#include "Planet.h"

#include <osg/ShapeDrawable>
#include <osg/TexGen>

using namespace pbs17;


/**
* \brief Constructor of SpaceObject.
*
* \param size
*      Size of the planet.
* \param mass
*      Mass of the planet.
* \param filename
*      Relative location to the object-file. (Relative from the data-directory in the source).
* \param translate
*      Initial translation of the object.
* \param center
*      Center of the global-rotation.
* \param scaling
*      Scaling of the model. (1.0 => not scaled, < 1.0 => smaller, > 1.0 => larger)
*/
Planet::Planet(double size, double mass, osg::Vec3 translate, osg::Vec3 center, double scaling)
	: SpaceObject("", center, scaling), _size(size), _mass(mass) {
	init(translate);
}


Planet::~Planet() {}


/**
* \brief Initialize the space-object for OSG.
*
* \param translate
*      Initial translation of the object.
*/
void Planet::init(osg::Vec3 translate) {
	// Load the model
	osg::ref_ptr<osg::ShapeDrawable> model = new osg::ShapeDrawable;
	model->setShape(new osg::Sphere(osg::Vec3(0.0f, 0.0f, 0.0f), _size));
	model->setColor(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));

	// First transformation-node to handle locale-rotations easier
	_rotation = new osg::MatrixTransform;
	_rotation->addChild(model);

	// Second transformation-node for global rotations and translations
	_model = new osg::MatrixTransform;
	_model->setMatrix(osg::Matrix::translate(translate));
	_model->addChild(_rotation);
}