#pragma once

#include "SpaceObject.h"

using json = nlohmann::json;
namespace pbs17 {

	/**
	 * \brief Class which represents any space-object with all relevant information needed for calculations
	 */
	class SpaceShip : public SpaceObject {
	public:
		/**
		 * \brief Constructor of SpaceShip.
		 */
        explicit SpaceShip();


		/**
		 * \brief Constructor of SpaceShip with JSON-configuration.
		 *
		 * \param j
		 *      JSON-configuration for the spaceship.
		 */
        explicit SpaceShip(json j);


		/**
		 * \brief Destructor of SpaceShip.
		 */
		~SpaceShip();


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
		void initPhysics(double mass, Eigen::Vector3d linearVelocity, Eigen::Vector3d angularVelocity, Eigen::Vector3d force, Eigen::Vector3d torque) override;


		void turnUp();

		void turnDown();

		void turnLeft();

		void turnRight();

		void accelerate();

		void decelerate();

	private:
        double _acceleration = 1.2;
        double _decelerate = 0.8;
        double _rotationAngle = 0.1; // randians ~ 5.7 degrees
		double intensity = 1.0;
	};

}
