#ifndef SUN_H
#define SUN_H

#include "Planet.h"

namespace pbs17 {

    /**
     * \brief Class which represents any space-object with all relevant information needed for calculations
     */
    class Sun : public Planet {
    public:
        /**
         * \brief Constructor of a Sun.
         *
         * \param size
         *      Size of the planet.
         */
        Sun(double size);

        /**
         * \brief Constructor of Planet with JSON-configuration.
         *
         * \param j
         *      JSON-configuration for the planet.
         */
        Sun(json j);

		void initTexturing() override;


    private:

    };

}

#endif // SUN_H

