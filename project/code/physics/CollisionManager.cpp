#include <iostream>
#include "CollisionManager.h"

using namespace pbs17;
using namespace Eigen;

CollisionManager::CollisionManager(std::vector<SpaceObject*> spaceObjects) {
	xList = std::vector<SpaceObject*>(spaceObjects);
	yList = std::vector<SpaceObject*>(spaceObjects);
	zList = std::vector<SpaceObject*>(spaceObjects);
	
	// sort the lists
	insertionSort(xList, 0);
	insertionSort(yList, 1);
	insertionSort(zList, 2);
}


/*
 * \brief Sort the space-objects based on the minimum coordinate on the given axis.
 * For this, the insertion-sort is used, since the relative order should normaly not change a lot.
 * Therefore it is mostly in O(n) to go through the list.
 * 
 * \param A
 *      Output-parameter:
 *			Input:	Vector with all possibly unsorted space-objects.
 *			Output: Vector with all space-objects sorted so that: for each i => i.min[dim] <= i+1.min[dim].
 * \param dim
 *	    Current axis for which the sort-criteria is checked.
 */
void CollisionManager::insertionSort(std::vector<SpaceObject*> &A, int dim) const {
	for (int i = 1; i < A.size(); ++i) {
		SpaceObject* tmp = A[i];
		int j = i - 1;

		while (j >= 0 && A[j]->getAABB()._min[dim] > A[j + 1]->getAABB()._min[dim]) {
			std::swap(A[j], A[j + 1]);
			j--;
		}

		A[j + 1] = tmp;
	}
}


/**
 * 
 */
void CollisionManager::handleCollisions(double dt, std::vector<SpaceObject *> _spaceObjects) {
	std::vector<std::pair<SpaceObject *, SpaceObject *>> collision;

	for (int i = 0; i < _spaceObjects.size(); ++i) {
		_spaceObjects[i]->resetCollisionState();
	}

	this->broadPhase(collision);
	this->narrowPhase(collision);
	this->respondToCollisions();
}


/**
 * \brief Find possible collisions based on the bounding-boxes of the objects.
 * The algorithm runs in O(n + #collisions), but the objects need to be sorted based on the min-edge.
 *
 * \param A
 *      Vector with all space-objects. Needs to be sorted based on i.min[dim] <= i+1.min[dim].
 * \param dim
 *	    Current axis for which the collisions checked.
 * \param res
 *	    Output-parameter: Vector with possible collisions. The value is a pair with the two objects which possibly colided.
 *	                      By convention, on the first position the object with the smaller id is stored (key.first.Id < key.second.Id)
 */
void CollisionManager::pruneAndSweep(std::vector<SpaceObject*> &A, int dim, std::vector<std::pair<SpaceObject *, SpaceObject *>> &res) {
	for (int i = 0; i < A.size() - 1; ++i) {
		double aabbMax = A[i]->getAABB()._max[dim];

		for (int j = i + 1; j < A.size(); ++j) {
			// If the left edge of the neighbor is smaller than the right edge of the current, a possible collision exists.
			if (aabbMax >= A[j]->getAABB()._min[dim]) {
				// always have the object with the smaller id first
				if (A[i]->getId() < A[j]->getId()) {
					res.push_back(std::make_pair(A[i], A[j]));
				} else {
					res.push_back(std::make_pair(A[j], A[i]));
				}
			} else {
				// Because the list is ordered for the min-edge (left), as soon as the neigbour is greater, we can
				// stop the search for possible collisions.
				break;
			}
		}
	}
}

void CollisionManager::broadPhase(std::vector<std::pair<SpaceObject *, SpaceObject *>> &res) {
	// Resort the list
	insertionSort(xList, 0);
	insertionSort(yList, 1);
	insertionSort(zList, 2);

	// Vectors which contains the possible collisions per axis
	std::vector<std::pair<SpaceObject *, SpaceObject *>> resX;
	std::vector<std::pair<SpaceObject *, SpaceObject *>> resY;
	std::vector<std::pair<SpaceObject *, SpaceObject *>> resZ;

	pruneAndSweep(xList, 0, resX);
	pruneAndSweep(yList, 1, resY);
	pruneAndSweep(zList, 2, resZ);

	std::sort(resX.begin(), resX.end());
	std::sort(resY.begin(), resY.end());
	std::sort(resZ.begin(), resZ.end());
	/*std::cout << "resX = " << resX.size() << std::endl;
	std::cout << "resY = " << resY.size() << std::endl;
	std::cout << "resZ = " << resZ.size() << std::endl;*/

	std::vector<std::pair<SpaceObject *, SpaceObject *>> resXY;

	// get only the instersections
	std::set_intersection(resX.begin(), resX.end(), resY.begin(), resY.end(),
		std::back_inserter(resXY));

	std::set_intersection(resXY.begin(), resXY.end(), resZ.begin(), resZ.end(),
		std::back_inserter(res));

	if (res.size() > 0) {
		std::cout << "Possible collisions: " << res.size() << std::endl;
	}
}

bool CollisionManager::checkIntersection(Planet *p1, Planet *p2) {
	double d = (p1->getPosition() - p2->getPosition()).norm();
	//std::cout << d << " " << p1->getRadius() << " " << p2->getRadius() << std::endl;
	return d < p1->getRadius() + p2->getRadius();
}

void CollisionManager::response(Planet *p1, Planet *p2) {
	Vector3d pos1 = p1->getPosition();
	Vector3d pos2 = p2->getPosition();

	// First, find the vector which will serve as a basis vector (x-axis),
	// in an arbitrary direction. It has to be normalized to get realistic results.
	Vector3d x = (pos1 - pos2).normalized();

	// Then we calculate the x-direction velocity vector and the perpendicular y-vector.
	Vector3d v1 = p1->getLinearVelocity();
	double x1 = x.dot(v1);
	Vector3d v1x = x * x1;
	Vector3d v1y = v1 - v1x;
	double m1 = p1->getMass();

	// Same procedure for the other sphere.
	Vector3d v2 = p2->getLinearVelocity();
	double x2 = x.dot(v2);
	Vector3d v2x = x * x2;
	Vector3d v2y = v1 - v2x;
	double m2 = p2->getMass();

	p1->setLinearVelocity(v1x * (m1 - m2) / (m1 + m2) + v2x * (2. * m2) / (m1 + m2) + v1y);
	p2->setLinearVelocity(v1x * (2. * m1) / (m1 + m2) + v2x * (m2 - m1) / (m1 + m2) + v2y);
}

void CollisionManager::narrowPhase(std::vector<std::pair<SpaceObject *, SpaceObject *>> &collision) {
	for (int i = 0; i < collision.size(); ++i) {
		Planet* p1 = dynamic_cast<Planet*>(collision[i].first);
		Planet* p2 = dynamic_cast<Planet*>(collision[i].second);

		if (p1 != NULL && p2 != NULL) {
			// we have a possible collision between 2 spheres.

			if (checkIntersection(p1, p2)) {
				Collision sphereCollision;
				sphereCollision.setFirstObject(p1);
				sphereCollision.setSecondObject(p2);
				sphereCollision.setUnitNormal((p1->getPosition() - p2->getPosition()).normalized());
				sphereCollision.setFirstPOC(p1->getPosition() - p1->getRadius() * sphereCollision.getUnitNormal());
				sphereCollision.setSecondPOC(p1->getPosition() + p1->getRadius() * sphereCollision.getUnitNormal());
				sphereCollision.setIntersectionVector(sphereCollision.getUnitNormal() * (((p1->getPosition() - p2->getPosition()).norm()) - p1->getRadius() - p2->getRadius()));
				collisionQueue.push(sphereCollision);
				//std::cout << "INTERSECTION DETECTED" << std::endl;

				p1->setCollisionState(2);
				p2->setCollisionState(2);
			} else {
				p1->setCollisionState(1);
				p2->setCollisionState(1);
			}
		} else {
			//std::cout << "TBD: impl narrow collision check between aribary space objects" << std::endl;
			collision[i].first->setCollisionState(1);
			collision[i].second->setCollisionState(1);
		}
	}
}

void CollisionManager::respondToCollisions() {
	while (!collisionQueue.empty()) {
		Collision currentCollision = collisionQueue.top();
		collisionQueue.pop();


		Eigen::Vector3d velocityFirst = currentCollision.getFirstObject()->getLinearVelocity();
		Eigen::Vector3d velocitySecond = currentCollision.getSecondObject()->getLinearVelocity();

		double closingVelocity = (velocityFirst - velocitySecond).dot(currentCollision.getUnitNormal());
		double finalVelocity = -COEF_RESTITUTION * closingVelocity;

		double deltaVelocity = finalVelocity - closingVelocity;

		double linearResponsePerUnitImpulse = 1. / currentCollision.getFirstObject()->getMass();
		linearResponsePerUnitImpulse += 1. / currentCollision.getSecondObject()->getMass();

		Eigen::Vector3d impulseResponse = (deltaVelocity / linearResponsePerUnitImpulse) * currentCollision.getUnitNormal();

		currentCollision.getFirstObject()->setPosition(currentCollision.getFirstObject()->getPosition() + currentCollision.getIntersectionVector() / 2);
		currentCollision.getFirstObject()->setPosition(currentCollision.getFirstObject()->getPosition() - currentCollision.getIntersectionVector() / 2);

		currentCollision.getFirstObject()->setLinearVelocity(currentCollision.getFirstObject()->getLinearVelocity() + impulseResponse / currentCollision.getFirstObject()->getMass());
		currentCollision.getSecondObject()->setLinearVelocity(currentCollision.getSecondObject()->getLinearVelocity() - impulseResponse / currentCollision.getSecondObject()->getMass());
	}
}
