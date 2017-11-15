#include "CollisionManager.h"

using namespace pbs17;

CollisionManager::CollisionManager() { }

void CollisionManager::handleCollisions(double dt, std::vector<SpaceObject *> _spaceObjects) {
    this->broadPhase();
    this->narrowPhase();
}

void CollisionManager::broadPhase() {

}

void CollisionManager::narrowPhase() {

}
