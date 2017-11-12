#pragma once

#include <vector>

#include "../scene/SpaceObject.h"

namespace pbs17 {

	/**
	 * \brief The manager to handle the simulation (within all needed calculations) to simulate one frame.
	 */
	class SimulationManager
	{
	public:
		/**
		* \brief Constructor of the simulation-manager.
		*
		* \param spaceObjects
		*      All space-objects in the scene.
		*/
		SimulationManager(std::vector<SpaceObject*> spaceObjects);


		/**
		* \brief Simulate the scene.
		*
		* \param dt
		*      Time difference since between the last frames.
		*/
		void simulate(double dt);

	private:
		//! All space-objects in the scene
		std::vector<SpaceObject*> _spaceObjects;
	};

}
