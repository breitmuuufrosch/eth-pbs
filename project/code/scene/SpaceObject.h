#pragma once

#include <osg/Node>
#include <Eigen/Core>
#include <osg/MatrixTransform>

namespace pbs17 {

	/**
	 * \brief Class which represents any space-object with all relevant information needed for calculations
	 */
	class SpaceObject {
	public:
		/**
		 * \brief Constructor of SpaceObject.
		 * 
		 * \param filename
		 *      Relative location to the object-file. (Relative from the data-directory in the source).
		 * \param center
		 *      Center of the global-rotation.
		 */
		SpaceObject(std::string filename, Eigen::Vector3d center);

		virtual ~SpaceObject();


		/**
		 * \brief Initialize the space-object for OSG.
		 * 
		 * \param position
		 *      Initial position of the object.
		 * \param scaling
		 *      Scaling of the model. (1.0 => not scaled, < 1.0 => smaller, > 1.0 => larger)
		 */
		virtual void initOsg(Eigen::Vector3d position, double scaling) = 0;


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
		virtual void initPhysics(double mass, double linearMomentum, double angularMomentum, double linearVelocity, double angularVelocity, Eigen::Vector3d force, Eigen::Vector3d torque);


		/**
		 * \brief Get the osg-node of the space-object.
		 * 
		 * \return OSG-node of the space-object.
		 */
		osg::ref_ptr<osg::MatrixTransform> getModel() const {
			return _model;
		}


		/**
		 * \brief Get the rotation-center of the space-object.
		 * 
		 * \return Position of the rotation-center.
		 */
		Eigen::Vector3d getCenter() const {
			return _center;
		}


		/**
		 * \brief Set the local rotation of the model. (Around it's local axis).
		 * 
		 * \param angle
		 *      Angle of the rotation.
		 * \param axis
		 *      Axis of the rotation.
		 */
		virtual void setLocalRotation(double angle, osg::Vec3 axis) const;


	protected:
		///! Filename of the loaded object
		std::string _filename;


		///! Root of the model which is used for the scene
		osg::ref_ptr<osg::MatrixTransform> _model;
		///! Local-rotation-node for the object
		osg::ref_ptr<osg::MatrixTransform> _rotation;

		///! Scaling ratio
		double _scaling;
		///! Position
		Eigen::Vector3d _position;
		///! Rotation-center of the object
		Eigen::Vector3d _center;

		///! Mass: unit = kg
		double _mass;
		///! Linear momentum : unit = kg*m / s
		double _linearMomentum;
		///! Angular Momentum : unit = kg*m ^ 2 / s
		double _angularMomentum;
		///! Linear velocity : unit = m / s
		double _linearVelcoity;
		///! Angular velocity : unit = rad / s
		double _angularVelocity;
		///! Global force : unit = vector with norm equals to N
		Eigen::Vector3d _force;
		///! Global torque : unit = vector with norm equals to N*m(newton metre)
		Eigen::Vector3d _torque;
	};

}
