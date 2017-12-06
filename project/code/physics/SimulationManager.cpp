#include "SimulationManager.h"

using namespace pbs17;

bool SimulationManager::IS_PAUSED = false;
double SimulationManager::SIMULATION_DT = 0.01;;

/**
 * \brief Constructor of the simulation-manager.
 * 
 * \param spaceObjects
 *      All space-objects in the scene.
 */
SimulationManager::SimulationManager(std::vector<SpaceObject*> spaceObjects)
	: _spaceObjects(spaceObjects) {
    _nManager = new NBodyManager();
    _cManager = new CollisionManager(spaceObjects);
}


/**
 * \brief Simulate the scene.
 *
 * \param dt
 *      Time difference since between the last frames.
 */
void SimulationManager::simulate(double dt) {
	if (IS_PAUSED) {
		return;
	}

    // simulate on step
    _nManager->simulateStep(dt, this->_spaceObjects);

    // check for collisions
    _cManager->handleCollisions(dt, this->_spaceObjects);

    // TBD: check for fraction
}
