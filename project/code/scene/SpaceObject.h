#pragma once

#include <osg/Node>
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
		 * \param scaling
		 *      Scaling of the model. (1.0 => not scaled, < 1.0 => smaller, > 1.0 => larger)
		 */
		SpaceObject(std::string filename, osg::Vec3 center, double scaling);

		virtual ~SpaceObject();


		/**
		 * \brief Initialize the space-object for OSG.
		 * 
		 * \param translate
		 *      Initial translation of the object.
		 */
		virtual void init(osg::Vec3 translate) = 0;


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
		osg::Vec3 getCenter() const {
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

		///! Scaling ratio
		double _scaling;

		///! Root of the model which is used for the scene
		osg::ref_ptr<osg::MatrixTransform> _model;
		///! Local-rotation-node for the object
		osg::ref_ptr<osg::MatrixTransform> _rotation;
		///! Rotation-center of the object
		osg::Vec3 _center;


		/**
		* \brief Scales a OSG-node to the desired size (by ratio). All axis are scaled uniformly.
		*
		* \param node
		*      Node which is scaled.
		* \param scaling
		*      Scaling-factor.
		*
		* \return Scaled OSG-node.
		*/
		static osg::ref_ptr<osg::MatrixTransform> scaleNode(osg::ref_ptr<osg::Node> node, double scaling);
	};

}