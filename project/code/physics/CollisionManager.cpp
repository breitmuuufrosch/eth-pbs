#include "CollisionManager.h"

using namespace pbs17;

CollisionManager::CollisionManager(std::vector<SpaceObject*> spaceObjects) {
    for (std::vector<SpaceObject*>::iterator it = spaceObjects.begin(); it != spaceObjects.end(); ++it) {
        SpaceObject* spaceObject = *it;

        xList.push_back(spaceObject);
        yList.push_back(spaceObject);
        zList.push_back(spaceObject);
    }

    // sort the lists
    insertionSort(xList, 0);
    insertionSort(yList, 1);
    insertionSort(zList, 2);
}
/*
 *
 *
 *
 */
void insertionSort(std::vector<SpaceObject*> &A, int dim) {
    int i = 0;
    while(i < l.size()) {
        int j = 0;
        while(j > 0 && A[j-1].getAABB().min[dim] > A[j].getAABB().min[dim]) {
            std::swap(A[j].getAABB().min[dim], A[j-1].getAABB().min[dim]);
            j--;
        }
        i++;
    }
}

void CollisionManager::handleCollisions(double dt, std::vector<SpaceObject *> _spaceObjects) {
    this->broadPhase();
    this->narrowPhase();
}

void CollisionManager::broadPhase() {

}

void CollisionManager::narrowPhase() {

}
