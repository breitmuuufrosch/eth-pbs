#include "SimulationManager.h"

#include "../osg/OsgEigenConversions.h"

using namespace pbs17;

/**
 * \brief Constructor of the simulation-manager.
 * 
 * \param spaceObjects
 *      All space-objects in the scene.
 */
SimulationManager::SimulationManager(std::vector<SpaceObject*> spaceObjects)
	: _spaceObjects(spaceObjects) {
}

/**
 * \brief Simulate the scene.
 *
 * \param dt
 *      Time difference since between the last frames.
 */
void SimulationManager::simulate(double dt) {
	for (std::vector<SpaceObject*>::iterator it = _spaceObjects.begin(); it != _spaceObjects.end(); ++it) {
		SpaceObject* spaceObject = *it;

		// Calculate a rotation around the rotation-center of the object
		osg::Vec3 toCenter = toOsg((*it)->getCenter());
		osg::Matrix rotationGlobal = osg::Matrix::rotate(dt, osg::Vec3(0.0f, 0.0f, 1.0f));
		osg::Matrixd translate1 = osg::Matrixd::translate(-toCenter);
		osg::Matrixd translate2 = osg::Matrixd::translate(toCenter);

		// Apply rotation to the current position and also rotate the object localy (rotation around it's own axis)
		spaceObject->getModel()->setMatrix(translate1 * rotationGlobal * translate2 * spaceObject->getModel()->getMatrix());
		spaceObject->setLocalRotation(-dt * 10, osg::Vec3(1.0f, 1.0f, 0.0f));
	}
}
