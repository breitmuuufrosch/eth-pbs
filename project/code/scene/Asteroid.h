#pragma once

#include "SpaceObject.h"

using json = nlohmann::json;
namespace pbs17 {

	/**
	 * \brief Class which represents any space-object with all relevant information needed for calculations
	 */
	class Asteroid : public SpaceObject {
	public:
		/**
		 * \brief Constructor of Asteroid.
		 * 
		 * \param filename
		 *      Relative location to the object-file. (Relative from the data-directory in the source).
		 */
        explicit Asteroid();
        explicit Asteroid(json j);


		/**
		 * \brief Destructor of Asteroid.
		 */
		~Asteroid();


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
		void initOsg(Eigen::Vector3d position, double ratio, double scaling) override;


		void setOrientation(Eigen::Vector3d o) override;
	};

}
