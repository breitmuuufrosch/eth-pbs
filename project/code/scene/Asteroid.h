#pragma once

#include <osg/Node>
#include <osg/MatrixTransform>

#include "SpaceObject.h"

namespace pbs17 {

	/**
	 * \brief Class which represents any space-object with all relevant information needed for calculations
	 */
	class Asteroid : public SpaceObject {
	public:
		/**
		 * \brief Constructor of SpaceObject.
		 * 
		 * \param filename
		 *      Relative location to the object-file. (Relative from the data-directory in the source).
		 * \param translate
		 *      Initial translation of the object.
		 * \param center
		 *      Center of the global-rotation.
		 * \param scaling
		 *      Scaling of the model. (1.0 => not scaled, < 1.0 => smaller, > 1.0 => larger)
		 */
		Asteroid(std::string filename, osg::Vec3 translate, osg::Vec3 center, double scaling);


		~Asteroid();


		/**
		 * \brief Initialize the space-object for OSG.
		 * 
		 * \param translate
		 *      Initial translation of the object.
		 */
		void init(osg::Vec3 translate);

	private:
		
	};

}
