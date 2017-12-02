#pragma once

#include <vector>
#include <queue>

#include "../scene/SpaceObject.h"
#include "../scene/Planet.h"
#include "Collision.h"
#include <Eigen/Core>

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
        * \param spaceObjects
        *      All space-objects in the scene.
        */
        void handleCollisions(double dt, std::vector<SpaceObject*> &spaceObjects);

    private:

        void broadPhase(std::vector<std::pair<SpaceObject *, SpaceObject *>> &res);
        void narrowPhase(std::vector<std::pair<SpaceObject *, SpaceObject *>> &collisions);
        void insertionSort(std::vector<SpaceObject *> &A, int dim) const;
		void respondToCollisions();

	    static bool checkIntersection(Planet *p1, Planet *p2);
	    static void response(Planet *p1, Planet *p2);
	    static void pruneAndSweep(std::vector<SpaceObject*> &A, int dim, std::vector<std::pair<SpaceObject *, SpaceObject *>> &res);

		static Eigen::Matrix3d getOrthonormalBasis(Eigen::Vector3d v);

        //! All space-objects in the scene
        std::vector<SpaceObject*> _xList;
        std::vector<SpaceObject*> _yList;
        std::vector<SpaceObject*> _zList;

		std::priority_queue<Collision, std::vector<Collision>, CollisionCompareLess> _collisionQueue;

        const double COEF_RESTITUTION = 1.0;
		const double COEF_FRICTION = 0.2;
    };

}
