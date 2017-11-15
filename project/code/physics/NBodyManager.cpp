#include "NBodyManager.h"

using namespace pbs17;

NBodyManager::NBodyManager() { }

void NBodyManager::simulateStep(double dt, std::vector<SpaceObject *> _spaceObjects) {
    for (std::vector<SpaceObject*>::iterator it = _spaceObjects.begin(); it != _spaceObjects.end(); ++it) {
        SpaceObject* spaceObject = *it;

        // Calculate a rotation around the rotation-center of the object
        osg::Vec3d toCenter = toOsg((*it)->getCenter());
        osg::Matrix rotationGlobal = osg::Matrix::rotate(dt, osg::Vec3d(0.0f, 0.0f, 1.0f));
        osg::Matrixd translate1 = osg::Matrixd::translate(-toCenter);
        osg::Matrixd translate2 = osg::Matrixd::translate(toCenter);

        // Apply rotation to the current position and also rotate the object localy (rotation around it's own axis)
        spaceObject->getModel()->setMatrix(translate1 * rotationGlobal * translate2 * spaceObject->getModel()->getMatrix());
        spaceObject->setLocalRotation(-dt * 10, osg::Vec3d(1.0f, 1.0f, 0.0f));
    }
}
