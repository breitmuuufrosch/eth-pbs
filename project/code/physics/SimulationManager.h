/**
 * \brief Implementation of the simulation manager which is called each frame.
 *
 * \Author: Alexander Lelidis (14-907-562), Andreas Emch (08-631-384), Uroš Tešić (17-950-346)
 * \Date:   2017-11-11
 */

#pragma once

#include <vector>
#include <algorithm>


// forward declarations
namespace pbs17 {
	class NBodyManager;
	class CollisionManager;
	class SpaceObject;
}


namespace pbs17 {

	/**
	 * \brief The manager to handle the simulation (within all needed calculations) to simulate one frame.
	 */
	class SimulationManager {
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

		static void setIsPaused(const bool isPaused) {
			IS_PAUSED = isPaused;
		}

		static bool getIsPaused() {
			return IS_PAUSED;
		}

		static void setSimulationDt(const double dt) {
			SIMULATION_DT = std::max(dt, 0.0);
			SIMULATION_DT = std::min(SIMULATION_DT, 1.0);
		}

		static void increaseSimulationDt(const double dt) {
			setSimulationDt(SIMULATION_DT + dt);
		}

		static void decreaseSimulationDt(const double dt) {
			setSimulationDt(SIMULATION_DT - dt);
		}

		static double getSimulationDt() {
			return SIMULATION_DT;
		}

	private:
		//! All space-objects in the scene
		std::vector<SpaceObject*> _spaceObjects;
		CollisionManager* _cManager;
		NBodyManager* _nManager;

		static bool IS_PAUSED;

		static double SIMULATION_DT;
	};
}
