#include "SpaceObject.h"

#include <osgDB/ReadFile>
#include <osg/PolygonMode>
#include <osg/ShapeDrawable>
#include "../osg/visitors/BoundingBoxVisitor.h"

using namespace pbs17;


//! ID's start from 0.
long SpaceObject::RunningId = 0;


/**
* \brief Constructor of SpaceObject.
*
* \param filename
*      Relative location to the object-file. (Relative from the data-directory in the source).
*/
SpaceObject::SpaceObject(std::string filename)
	: SpaceObject(filename, "") {}


/**
* \brief Constructor of SpaceObject.
*
* \param filename
*      Relative location to the object-file. (Relative from the data-directory in the source).
* \param textureName
*      Relative location to the texture-file. (Relative from the data-directory in the source).
*/
SpaceObject::SpaceObject(std::string filename, std::string textureName)
	: _filename(filename), _textureName(textureName) {
	_id = RunningId;
	++RunningId;

	_position = Eigen::Vector3d(0, 0, 0);
	_orientation = Eigen::Vector3d(0, 0, 0);

	// For visually debuggin => Make bounding-box visible
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	_aabbShape = new osg::ShapeDrawable(new osg::Box(osg::Vec3(), 1.0f));
	_aabbShape->setColor(osg::Vec4(1.0, 0, 0, 1.0));
	geode->addDrawable(_aabbShape);

	_aabbRendering = new osg::MatrixTransform;
	_aabbRendering->setNodeMask(0x1);
	_aabbRendering->addChild(geode.get());
	osg::StateSet* ss = _aabbRendering->getOrCreateStateSet();
	ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	ss->setAttributeAndModes(new osg::PolygonMode(
		osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE));
}


/**
 * \brief Destructor of SpaceObject.
 */
SpaceObject::~SpaceObject() {
	if (_modelRoot) {
		_modelRoot = nullptr;
	}
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
void SpaceObject::initPhysics(double mass, Eigen::Vector3d linearVelocity, Eigen::Vector3d angularVelocity, Eigen::Vector3d force, Eigen::Vector3d torque) {
	_mass = mass;
	_linearVelocity = linearVelocity;
	_angularVelocity = angularVelocity;
	_force = force;
	_torque = torque;
}


void SpaceObject::updatePositionOrientation(Eigen::Vector3d p, Eigen::Vector3d dtv, Eigen::Vector3d o, Eigen::Vector3d dto) {
	_position = p;
	_orientation = o;

	_translation->setMatrix(osg::Matrix::translate(toOsg(p)));
	_rotation->setMatrix(osg::Matrixd::rotate(osg::Quat(o[0], osg::X_AXIS, o[1], osg::Y_AXIS, o[2], osg::Z_AXIS)));

	auto rotation = fromOsg(osg::Matrixd::rotate(osg::Quat(dto[0], osg::X_AXIS, dto[1], osg::Y_AXIS, dto[2], osg::Z_AXIS)));

	// Then obtain the property map for P
	Aff_transformation_3 trans1(
		rotation(0, 0), rotation(0, 1), rotation(0, 2), dtv(0),
		rotation(1, 0), rotation(1, 1), rotation(1, 2), dtv(1),
		rotation(2, 0), rotation(2, 1), rotation(2, 2), dtv(2),
		1);
	_convexHull->transform(trans1);

	calculateAABB();
}


void SpaceObject::calculateAABB() {
	osg::Matrix scaling = osg::Matrix::scale(_scaling, _scaling, _scaling);
	osg::Matrix translation = osg::Matrix::translate(toOsg(_position));
	osg::Matrix rotation = osg::Matrixd::rotate(osg::Quat(_orientation[0], osg::X_AXIS, _orientation[1], osg::Y_AXIS, _orientation[2], osg::Z_AXIS));

	CalculateBoundingBox bbox(scaling * rotation * translation, scaling * rotation);
	_modelFile->accept(bbox);
	_aabbLocal = bbox.getLocalBoundBox();
	_aabbGlobal = bbox.getGlobalBoundBox();

	_aabbRendering->setMatrix(osg::Matrix::scale(_aabbGlobal._max - _aabbGlobal._min) * osg::Matrix::translate(toOsg(_position)));
}


void SpaceObject::resetCollisionState() {
	if (_collisionState == 0) {
		//ColorVisitor colorVisitor(osg::Vec4(1, 1, 1, 1));
		//_aabbRendering->accept(colorVisitor);
		_aabbShape->setColor(osg::Vec4(1, 1, 1, 1));
	}

	_collisionState = 0;
}

void SpaceObject::setCollisionState(int c) {
	_collisionState = std::max(_collisionState, c);

	//ColorVisitor colorVisitor(c == 1 ? osg::Vec4(0, 1, 0, 1) : osg::Vec4(1, 0, 0, 1));
	_aabbShape->setColor(c == 1 ? osg::Vec4(0, 1, 0, 1) : osg::Vec4(1, 0, 0, 1));
	//_aabbRendering->accept(colorVisitor);
}