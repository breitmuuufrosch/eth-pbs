#include "Sun.h"


using namespace pbs17;


/**
 * \brief Constructor of Sun.
 *
 * \param size
 *      Size of the planet.
 */
Sun::Sun(double size)
    : Planet(size) {
}


Sun::Sun(json j)
    : Planet(j) {

}
