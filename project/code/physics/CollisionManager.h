#pragma once

#include <vector>

#include "../scene/SpaceObject.h"

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
        CollisionManager();


        /**
        * \brief Simulate the scene.
        *
        * \param dt
        *      Time difference since between the last frames.
        */
        void handleCollisions(double dt, std::vector<SpaceObject*> _spaceObjects);

    private:
        //! All space-objects in the scene
        void broadPhase();
        void narrowPhase();
    };

}
