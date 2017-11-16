#pragma once

#include <vector>
#include <queue>

#include "../scene/SpaceObject.h"
#include "../scene/Planet.h"
#include "Collision.h"
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>

namespace pbs17 {

    /**
     * \brief The manager to handle the collision (within all needed calculations) to simulate one frame.
     */
    class CollisionManager
    {
    public:
        /**
        * \brief Constructor of the CollisionManager.
        *
        */
        CollisionManager(std::vector<SpaceObject*> _spaceObjects);


        /**
        * \brief Simulate the scene.
        *
        * \param dt
        *      Time difference since between the last frames.
        */
        void handleCollisions(double dt, std::vector<SpaceObject*> _spaceObjects);

    private:
        //! All space-objects in the scene
        void broadPhase(std::vector<std::pair<SpaceObject *, SpaceObject *>> &res);
        void narrowPhase(std::vector<std::pair<SpaceObject *, SpaceObject *>> &collision);
        void insertionSort(std::vector<SpaceObject *> &A, int dim);
		void respondToCollisions();

        bool checkIntersection(Planet *p1, Planet *p2);
        void response(Planet *p1, Planet *p2);
        void pruneAndSweep(std::vector<SpaceObject*> &A, int dim, std::vector<std::pair<SpaceObject *, SpaceObject *>> &res);

		Eigen::Matrix3d getOrthonormalBasis(Eigen::Vector3d v);

        std::vector<SpaceObject*> xList;
        std::vector<SpaceObject*> yList;
        std::vector<SpaceObject*> zList;

		std::priority_queue<Collision, std::vector<Collision>, CollisionCompareLess> collisionQueue;

		const double COEF_RESTITUTION = 1.;
		const double COEF_FRICTION = 0.2;
    };

}
