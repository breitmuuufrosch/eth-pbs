#pragma once

#include <Eigen/Core>
#include <osg/Switch>
#include <osg/MatrixTransform>
#include <osg/BoundingBox>
#include <osg/ShapeDrawable>
#include <json.hpp>
#include <string>

#include "../osg/visitors/BoundingBoxVisitor.h"
#include "../osg/OsgEigenConversions.h"
#include "../graphics/ConvexHull3D.h"

using json = nlohmann::json;


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
        SpaceObject(std::string filename, int i);
        SpaceObject(json j);


		/**
		 * \brief Constructor of SpaceObject.
		 *
		 * \param filename
		 *      Relative location to the object-file. (Relative from the data-directory in the source).
		 * \param textureName
		 *      Relative location to the texture-file. (Relative from the data-directory in the source).
		 */
		SpaceObject(std::string filename, std::string textureName);


		/**
		 * \brief Destructor.
		 */
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
		osg::ref_ptr<osg::Switch> getModel() const {
			return _modelRoot;
		}

		osg::ref_ptr<osg::Switch> getConvexSwitch() const {
			return _convexRenderSwitch;
		}

		osg::ref_ptr<osg::MatrixTransform> getTranslation() const
		{
			return _translation;
		}

		osg::ref_ptr<osg::MatrixTransform> getRotation() const {
			return _rotation;
		}


		/**
		 * \brief Get the AABB of the space-object.
		 */
		osg::BoundingBox getAABB() const {
			return _aabbGlobal;
		}


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

		Eigen::Vector3d getPosition() const {
			return _position;
		}

		void setPosition(Eigen::Vector3d p) {
			_position = p;
			_translation->setMatrix(osg::Matrix::translate(toOsg(p)));
			calculateAABB();
		}

		Eigen::Vector3d getLinearVelocity() const {
			return _linearVelocity;
		}

		void setLinearVelocity(Eigen::Vector3d v) {
			_linearVelocity = v;
		}

		Eigen::Matrix3d getMomentOfInertia() const {
			return _momentOfInertia;
		}

		void setMomentOfInertia(Eigen::Matrix3d m) {
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

		void setOrientation(Eigen::Vector3d o) {
			_orientation = o;
			_rotation->setMatrix(osg::Matrixd::rotate(osg::Quat(o[0], osg::X_AXIS, o[1], osg::Y_AXIS, o[2], osg::Z_AXIS)));
			calculateAABB();
		}

		void updatePositionOrientation(Eigen::Vector3d p, Eigen::Vector3d dtv, Eigen::Vector3d o, Eigen::Vector3d dto);

		void calculateAABB();

		void resetCollisionState();

		void setCollisionState(int c);

		const ConvexHull3D* getConvexHull() const {
			return _convexHull;
		}


	protected:

		//! Filename of the loaded object
		std::string _filename;
		//! Filename of the loaded texture
		std::string _textureName = "";
		//! Unique identifier for the object
		long _id;


		//! Root of the model which is used for the scene
		osg::ref_ptr<osg::Switch> _modelRoot;
		osg::ref_ptr<osg::Switch> _convexRenderSwitch;
		osg::ref_ptr<osg::Node> _modelFile;
		osg::ref_ptr<osg::MatrixTransform> _aabbRendering;
		osg::ref_ptr<osg::ShapeDrawable> _aabbShape;
		//! Local-rotation-node for the object
		osg::ref_ptr<osg::MatrixTransform> _translation;
		osg::ref_ptr<osg::MatrixTransform> _rotation;

		//! Scaling ratio
		double _scaling;
		//! Position
		Eigen::Vector3d _position;
		//! Orientation
		Eigen::Vector3d _orientation;
		//! AABB of the object
		osg::BoundingBox _aabbLocal;
		osg::BoundingBox _aabbGlobal;
		//! ConvexHull of the object
		ConvexHull3D* _convexHull;

		//! Mass: unit = kg
		double _mass;
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

		int _collisionState;


	private:

		//! Running Id for uniquely identifying the objects.
		static long RunningId;
	};

}
