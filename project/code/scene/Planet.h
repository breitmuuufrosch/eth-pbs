#pragma once

#include <osg/Node>
#include <osg/MatrixTransform>

#include "SpaceObject.h"

namespace pbs17 {

	/**
	 * \brief Class which represents any space-object with all relevant information needed for calculations
	 */
	class Planet : public SpaceObject {
	public:
		/**
		 * \brief Constructor of SpaceObject.
		 * 
		 * \param size
		 *      Size of the planet.
		 * \param mass
		 *      Mass of the planet.
		 * \param translate
		 *      Initial translation of the object.
		 * \param center
		 *      Center of the global-rotation.
		 * \param scaling
		 *      Scaling of the model. (1.0 => not scaled, < 1.0 => smaller, > 1.0 => larger)
		 */
		Planet(double size, double mass, osg::Vec3 translate, osg::Vec3 center, double scaling);

		~Planet();


		/**
		 * \brief Initialize the space-object for OSG.
		 * 
		 * \param translate
		 *      Initial translation of the object.
		 */
		void init(osg::Vec3 translate) override;


	private:
		///! Size of the planet
		double _size;
		///! Mass of the planet
		double _mass;

	};

}
