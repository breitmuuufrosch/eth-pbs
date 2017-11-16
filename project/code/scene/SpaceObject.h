#pragma once

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <osg/MatrixTransform>
#include "../osg/visitors/BoundingBoxVisitor.h"
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
		 */
		SpaceObject(std::string filename);


        SpaceObject(std::string filename, std::string textureName);


		virtual ~SpaceObject();


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
		virtual void initOsg(Eigen::Vector3d position, double ratio, double scaling) = 0;


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
        virtual void initPhysics(double mass, Eigen::Vector3d linearVelocity, Eigen::Vector3d angularVelocity, Eigen::Vector3d force, Eigen::Vector3d torque);


		/**
		 * \brief Get the osg-node of the space-object.
		 * 
		 * \return OSG-node of the space-object.
		 */
		osg::ref_ptr<osg::MatrixTransform> getModel() const {
			return _model;
		}

		osg::ref_ptr<osg::MatrixTransform> getRotation() const{
			return _rotation;
		}

		void setOrientation(Eigen::Vector3d o) {
			_orientation = o;
		}


		/**
		 * \brief Get the AABB of the space-object.
		 */
		osg::BoundingBox getAABB() const {
            //return _aabb;
            CalculateBoundingBox bbox;
            _model->accept(bbox);
            return bbox.getBoundBox();
		}


		/**
		 * \brief Set the local rotation of the model. (Around it's local axis).
		 * 
		 * \param angle
		 *      Angle of the rotation.
		 * \param axis
		 *      Axis of the rotation.
		 */
		virtual void setLocalRotation(double angle, osg::Vec3d axis) const;


		/**
		 * \brief Get the ID of the object.
		 * 
		 * \return Unique identifier of the object.
		 */
		long getId() const {
			return _id;
		}

        double getMass() const {
            return _mass;
        }

        Eigen::Vector3d getLinearVelocity() const {
            return _linearVelocity;
        }

        Eigen::Vector3d getPosition() const {
            return _position;
        }

        void setPosition(Eigen::Vector3d p) {
            _position = p;
        }

        void setLinearVelocity(Eigen::Vector3d v) {
            _linearVelocity = v;
        }

		Eigen::Matrix3d getMomentOfInertia() const {
			return _momentOfInertia;
		}

		Eigen::Matrix3d setMomentOfInertia(Eigen::Matrix3d m) {
			_momentOfInertia = m;
		}

		Eigen::Vector3d getAngularVelocity() const {
			return _angularVelocity;
		}

		void setAngularVelocity(Eigen::Vector3d av) {
			_angularVelocity = av;
		}

		Eigen::Vector3d getOrientation() const {
			return _orientation;
		}


	protected:
		//! Filename of the loaded object
		std::string _filename;

        std::string _textureName = "";
		//! Unique identifier for the object
		long _id;


		//! Root of the model which is used for the scene
		osg::ref_ptr<osg::MatrixTransform> _model;
		//! Local-rotation-node for the object
		osg::ref_ptr<osg::MatrixTransform> _rotation;

		//! Scaling ratio
		double _scaling;
		//! Position
		Eigen::Vector3d _position;
		//! Orientation
		Eigen::Vector3d _orientation;
		//! AABB of the object
		osg::BoundingBox _aabb;

		//! Mass: unit = kg
		double _mass;
		//! Linear momentum : unit = kg*m / s
		Eigen::Vector3d _linearMomentum;
		//! Angular Momentum : unit = kg*m ^ 2 / s
		Eigen::Vector3d _angularMomentum;
		//! Linear velocity : unit = m / s
        Eigen::Vector3d _linearVelocity;
		//! Angular velocity : unit = rad / s
		Eigen::Vector3d _angularVelocity;
		//! Moment of inertia tensor (N.B. It is in the local coordinate system)
		Eigen::Matrix3d _momentOfInertia;
		//! Global force : unit = vector with norm equals to N
		Eigen::Vector3d _force;
		//! Global torque : unit = vector with norm equals to N*m(newton metre)
		Eigen::Vector3d _torque;

	private:
		//! Running Id for uniquely identifying the objects.
		static long RunningId;
	};

}
