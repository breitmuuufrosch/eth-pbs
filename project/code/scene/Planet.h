#pragma once

#include <osg/Node>

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
		 * \param center
		 *      Center of the global-rotation.
		 */
		Planet(double size, Eigen::Vector3d center);

		~Planet();


		/**
		 * \brief Initialize the space-object for OSG.
		 * 
		 * \param position
		 *      Initial translation of the object.
		 * \param scaling
		 *      Scaling of the model. (1.0 => not scaled, < 1.0 => smaller, > 1.0 => larger)
		 */
		void initOsg(Eigen::Vector3d position, double scaling) override;


	private:
		///! Size of the planet
		double _size;

	};

}
