#include "SimulationManager.h"

using namespace pbs17;

/**
 * \brief Constructor of the simulation-manager.
 * 
 * \param spaceObjects
 *      All space-objects in the scene.
 */
SimulationManager::SimulationManager(std::vector<SpaceObject*> spaceObjects)
	: _spaceObjects(spaceObjects) {
    nManager = new NBodyManager();
    cManager = new CollisionManager(spaceObjects);

}

/**
 * \brief Simulate the scene.
 *
 * \param dt
 *      Time difference since between the last frames.
 */
void SimulationManager::simulate(double dt) {
    // simulate on step
    this->nManager->simulateStep(dt, this->_spaceObjects);

    // check for collisions
    this->cManager->handleCollisions(dt, this->_spaceObjects);

    // TBD: check for fraction
}
