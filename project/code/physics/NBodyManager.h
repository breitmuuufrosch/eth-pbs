#pragma once

#include <vector>

#include "../scene/SpaceObject.h"

namespace pbs17 {

    /**
     * \brief The manager to handle the collision (within all needed calculations) to simulate one frame.
     */
    class NBodyManager
    {
    public:
        /**
        * \brief Constructor of the CollisionManager.
        *
        */
        NBodyManager();


        /**
        * \brief Simulate the scene.
        *
        * \param dt
        *      Time difference since between the last frames.
		* \param spaceObjects
		*      All space-objects in the scene.
        */
        void simulateStep(double dt, std::vector<SpaceObject*> &spaceObjects);

    };

}
