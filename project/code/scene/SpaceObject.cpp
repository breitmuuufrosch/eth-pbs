#include "SpaceObject.h"

#include <osgDB/ReadFile>

using namespace pbs17;


//! ID's start from 0.
long SpaceObject::RunningId = 0;


/**
* \brief Constructor of SpaceObject.
*
* \param filename
*      Relative location to the object-file. (Relative from the data-directory in the source).
* \param center
*      Center of the global-rotation.
*/
SpaceObject::SpaceObject(std::string filename, Eigen::Vector3d center)
	: _filename(filename), _center(center) {
	_id = RunningId;
	++RunningId;
}


/**
 * \brief Destructor of SpaceObject.
 */
SpaceObject::~SpaceObject() {
	if (_model) {
		_model = nullptr;
	}
}


/**
 * \brief Initialize the space-object for physics.
 *
 * \param mass
 *      Mass: unit = kg
 * \param linearMomentum
 *      Linear momentum: unit = kg*m/s
 * \param angularMomentum
 *      Angular Momentum: unit = kg*m^2/s
 * \param linearVelocity
 *      Linear velocity: unit = m/s
 * \param angularVelocity
 *      Angular velocity: unit = rad/s
 * \param force
 *      Global force: unit = vector with norm equals to N
 * \param torque
 *      Global torque: unit = vector with norm equals to N*m (newton metre)
 */
void SpaceObject::initPhysics(double mass, double linearMomentum, double angularMomentum, double linearVelocity, double angularVelocity, Eigen::Vector3d force, Eigen::Vector3d torque) {
	_mass = mass;
	_linearMomentum = linearMomentum;
	_angularMomentum = angularMomentum;
	_linearVelcoity = linearVelocity;
	_angularVelocity = angularVelocity;
	_force = force;
	_torque = torque;
}


/**
 * \brief Set the local rotation of the model. (Around it's local axis).
 *
 * \param angle
 *      Angle of the rotation.
 * \param axis
 *      Axis of the rotation.
 */
void SpaceObject::setLocalRotation(double angle, osg::Vec3d axis) const {
	osg::Matrix rotationMatrix = osg::Matrix::rotate(angle, axis);
	_rotation->setMatrix(rotationMatrix * _rotation->getMatrix());
}