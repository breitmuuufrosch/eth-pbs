#include "Vec2.h"


// gravitational acceleration (9.81)
static const double g = 9.81;
static const double kr = 100.0;


/**
 * \brief Calculate the external force of an object in the mass-spring-system
 * 
 * \param m Mass of the object
 * 
 * \return External force of the object
 */
double Fext(double m) {
    return m * - g;
}


/**
 * \brief Calculate the internal force of an object in the mass-spring-system
 * 
 * \param k  Stiffness of the spring
 * \param L  Length of the spring in rest-mode
 * \param p1 Position of point 1
 * \param p2 Position of point 2
 *
 * \return
 *		Internal force of the object
 */
double Fint(double k, double L, double p1, double p2) {
	
	return k * (L - fabs(p1 - p2));
}


/**
 * \brief Calculate the new state of the mass-spring-system with the Explicit Euler.
 * 
 * \param k  Stiffness of the spring
 * \param m  Mass of the objects (the same for all)
 * \param d  Damping factor
 * \param L  Length of the spring in rest-mode
 * \param dt Time-step
 * \param p1 Position of point 1 at beginning of time-frame
 * \param v1 Velocity of point 1 at beginning of time-frame
 * \param p2 Output-parameter
 *           Input:  Position of point 2 at beginning of time-frame
 *           Output: Position of point 2 after applying the time-step
 * \param v2 Output-parameter
 *           Input:  Velocity of point 2 at beginning of time-frame
 *           Output: Velocity of point 2 after applying the time-step
 */
void explicitEuler(double k, double m, double d, double L, double dt, double p1, double v1, double& p2, double& v2) {
    // use old value for velocity computation
    double p2Old = p2;
    p2 = p2 + dt * v2;
    v2 = v2 + dt * (1.0 / m) * (- Fint(k, L, p1, p2Old) + Fext(m) - d * v2);
}


/**
* \brief Calculate the new state of the mass-spring-system with the Symplectic Euler.
*
* \param k  Stiffness of the spring
* \param m  Mass of the objects (the same for all)
* \param d  Damping factor
* \param L  Length of the spring in rest-mode
* \param dt Time-step
* \param p1 Position of point 1 at beginning of time-frame
* \param v1 Velocity of point 1 at beginning of time-frame
* \param p2 Output-parameter
*           Input:  Position of point 2 at beginning of time-frame
*           Output: Position of point 2 after applying the time-step
* \param v2 Output-parameter
*           Input:  Velocity of point 2 at beginning of time-frame
*           Output: Velocity of point 2 after applying the time-step
*/
void symplecticEuler(double k, double m, double d, double L, double dt, double p1, double v1, double& p2, double& v2) {
    v2 = v2 + dt * (1.0 / m) * (- Fint(k, L, p1, p2) + Fext(m) - d * v2);
    p2 = p2 + dt * v2;
}


/**
* \brief Calculate the new state of the mass-spring-system with the Explicit Midpoint.
* \description This used the Explicit Euler to generate the midpoints
*
* \param k  Stiffness of the spring
* \param m  Mass of the objects (the same for all)
* \param d  Damping factor
* \param L  Length of the spring in rest-mode
* \param dt Time-step
* \param p1 Position of point 1 at beginning of time-frame
* \param v1 Velocity of point 1 at beginning of time-frame
* \param p2 Output-parameter
*           Input:  Position of point 2 at beginning of time-frame
*           Output: Position of point 2 after applying the time-step
* \param v2 Output-parameter
*           Input:  Velocity of point 2 at beginning of time-frame
*           Output: Velocity of point 2 after applying the time-step
*/
void midPoint(double k, double m, double d, double L, double dt, double p1, double v1, double& p2, double& v2) {
    double p2Midpoint = p2;
    double v2Midpoint = v2;

    // make an explicit euler step with half the step size
    explicitEuler(k, m, d, L, dt / 2.0, p1, v1, p2Midpoint, v2Midpoint);

    p2 = p2 + dt * v2Midpoint;
    v2 = v2 + dt * (1.0 / m) * (- Fint(k, L, p1, p2Midpoint) + Fext(m) - d * v2Midpoint);
}


/**
* \brief Calculate the new state of the mass-spring-system with the Semi-Implicit Euler.
*
* \param k  Stiffness of the spring
* \param m  Mass of the objects (the same for all)
* \param d  Damping factor
* \param L  Length of the spring in rest-mode
* \param dt Time-step
* \param p1 Position of point 1 at beginning of time-frame
* \param v1 Velocity of point 1 at beginning of time-frame
* \param p2 Output-parameter
*           Input:  Position of point 2 at beginning of time-frame
*           Output: Position of point 2 after applying the time-step
* \param v2 Output-parameter
*           Input:  Velocity of point 2 at beginning of time-frame
*           Output: Velocity of point 2 after applying the time-step
*/
void backwardEuler(double k, double m, double d, double L, double dt, double p1, double v1, double& p2, double& v2) {
	double p2_old = p2;
	double v2_old = v2;

	double p2Solution1 = d*dt*p2_old - g*dt*dt*m - dt*dt*k*L - dt*dt*k*p1 + dt*m*v2_old + m*p2_old;
	p2Solution1 /= (d*dt - dt*dt*k + m);

	double p2Solution2 = d*dt*p2_old - g*dt*dt*m - dt*dt*k*L + dt*dt*k*p1 + dt*m*v2_old + m*p2_old;
	p2Solution2 /= (d*dt + dt*dt*k + m);

	//if (p2Solution1 > p1) {
		//p2 = p2Solution1;
	//} else {
		//p2 = p2Solution2;
	//}

	if (p2Solution2 < p1) {
		p2 = p2Solution2;
	} else {
		p2 = p2Solution1;
	}

	v2 = (p2 - p2_old) / dt;
}


/**
* \brief Calculate the new state of the mass-spring-system analytically.
* \description Derived formulas can be found in the report.
*
* \param k  Stiffness of the spring
* \param m  Mass of the objects (the same for all)
* \param d  Damping factor
* \param L  Length of the spring in rest-mode
* \param t  Current time
* \param p1 Position of point 1 at beginning of time-frame
* \param v1 Velocity of point 1 at beginning of time-frame
* \param p2 Output-parameter
*           Input:  Position of point 2 at beginning of time-frame
*           Output: Position of point 2 after applying the time-step
* \param v2 Output-parameter
*           Input:  Velocity of point 2 at beginning of time-frame
*           Output: Velocity of point 2 after applying the time-step
*/
void analytic(double k, double m, double d, double L, double t, double p1, double v1, double& p2, double& v2) {
    double alpha = - d / (2.0 *  m);
    double beta = sqrt(4.0 * k * m - d * d) / (2.0 * m);
    double c_1 = m * g / k + L - 1;
    double c_2 = - c_1 * alpha / beta;

    p2 = c_1 * exp(alpha * t) * cos(beta * t) + c_2 * exp(alpha * t) * sin(beta *  t) - L - ((m * g) / k); 
    v2 = c_1 * exp(alpha * t) * (alpha * cos(beta * t) - beta * sin(beta * t)) + c_2 * exp(alpha * t) * (alpha * sin(beta * t) - beta * cos(beta * t));
}


/**
* \brief Exercise 1: hanging mass point
* \description Calculate the new state of the mass-spring-system by given method.
*              Supported methods are:
*               - explicit Euler       method = 1
*               - symplectic Euler     method = 2
*               - explicit midpoint    method = 3
*               - semi-implicit Euler  method = 4
*               - analytic             method = 5
*
* \param k  Stiffness of the spring
* \param m  Mass of the objects (the same for all)
* \param d  Damping factor
* \param L  Length of the spring in rest-mode
* \param dt Time-step or current time (for analytic-method)
* \param p1 Position of point 1 at beginning of time-frame
* \param v1 Velocity of point 1 at beginning of time-frame
* \param p2 Output-parameter
*           Input:  Position of point 2 at beginning of time-frame
*           Output: Position of point 2 after applying the time-step
* \param v2 Output-parameter
*           Input:  Velocity of point 2 at beginning of time-frame
*           Output: Velocity of point 2 after applying the time-step
*/
void AdvanceTimeStep1(double k, double m, double d, double L, double dt, int method, double p1, double v1, double& p2, double& v2) {
    switch(method) {
        case 1 : 
            explicitEuler(k, m, d, L, dt, p1, v1, p2, v2);
            break;
        case 2 : 
            symplecticEuler(k, m, d, L, dt, p1, v1, p2, v2);
            break;
        case 3 : 
            midPoint(k, m, d, L, dt, p1, v1, p2, v2);
            break;
        case 4 :
            backwardEuler(k, m, d, L, dt, p1, v1, p2, v2);
            break;
        case 5 :
            analytic(k, m, d, L, dt, p1, v1, p2, v2);
            break;
        default: 
            break;
    }
}


// -------------------------------------------------------------------------------------
// Exercise 3
// -------------------------------------------------------------------------------------


/**
* \brief Calculate the external force of an object in the mass-spring-system
* An external force is applied as soon as the point hit the floor at -1.
*
* \param m Mass of the object
* \param p Point to calculate the force for
*
* \return External force of the object
*/
Vec2 FextTri(double m, Vec2 p) { 
    double k_r = 0.0;
    // only aply forces if p.y is below the ground y = -1
    if(p.y < -1.0 ) {
        k_r = kr * (-1.0 - p.y);
    } 

    return Vec2(0.0, k_r + (m * (- g)));
}


/**
* \brief Calculate the internal force of an object in the mass-spring-system
*
* \param k  Stiffness of the spring
* \param L  Length of the spring in rest-mode
* \param p1 Position of point 1
* \param p2 Position of point 2
*
* \return
*		Internal force of the object
*/
Vec2 FintTri(double k, double L, Vec2 p1, Vec2 p2) {
    // compute the length 
    double y = (p2 - p1).length();

    // get the direction
    Vec2 direction = (1.0 / y) * (p2 - p1) ;
    
    return (k * (L - y) ) * direction;
}


/**
 * \brief Calculate all forces for a single point. All incident-springs need to be calculated as well.
 * The new position and velocity for the current concidered point will be applied on the output-parameters.
 * 
 * \param k  Stiffness of the spring
 * \param m  Mass of the objects (the same for all)
 * \param d  Damping factor
 * \param L  Length of the spring in rest-mode
 * \param dt Time-step
 * \param p1 Output parameter: Position of current point at the end of time-frame
 * \param v1 Output parameter: Velocity of current point at the end of time-frame
 * \param p1 Position of current point at beginning of time-frame
 * \param p1 Position of incident point 1 at beginning of time-frame
 * \param p1 Position of incident point 2 at beginning of time-frame
 */
void updateSingle(double k, double m, double d, double L, double dt,
				 Vec2& p1, Vec2& v1, Vec2& p1Old, Vec2& p2Old, Vec2& p3Old) {
	Vec2 Fint1 = FintTri(k, L, p1Old, p2Old);
	Fint1 += FintTri(k, L, p1Old, p3Old);

	Vec2 Fext1 = FextTri(m, p1Old);

	v1 = v1 + dt * (1.0 / m) * (Fext1 - Fint1 - d * v1);
	p1 = p1 + dt * v1;
}


/**
* \brief Calculate new positions and velocity for all points in the system.
*
* \param k  Stiffness of the spring
* \param m  Mass of the objects (the same for all)
* \param d  Damping factor
* \param L  Length of the spring in rest-mode
* \param dt Time-step
* \param p1 Output parameter: Position of point 1 at the end of time-frame
* \param v1 Output parameter: Velocity of point 1 at the end of time-frame
* \param p2 Output parameter: Position of point 2 at the end of time-frame
* \param v2 Output parameter: Velocity of point 2 at the end of time-frame
* \param p3 Output parameter: Position of point 3 at the end of time-frame
* \param v3 Output parameter: Velocity of point 3 at the end of time-frame
*/
void symplecticEuler(double k, double m, double d, double L, double dt,
                      Vec2& p1, Vec2& v1, Vec2& p2, Vec2& v2, Vec2& p3, Vec2& v3) {

    Vec2 p1Old = p1;
    Vec2 p2Old = p2;
    Vec2 p3Old = p3;

    // Apply smpletic-euler for each node
	updateSingle(k, m, d, L, dt, p1, v1, p1Old, p2Old, p3Old);
	updateSingle(k, m, d, L, dt, p2, v2, p2Old, p1Old, p3Old);
	updateSingle(k, m, d, L, dt, p3, v3, p3Old, p1Old, p2Old);
}

/**
 * \brief Exercise 3: falling triangle
 * The iteration over the springs is done in the update-single method itself, where for each mass
 * the external and all internal forces are calculated and applied.
 * The ground condition is handled in the calculation when the external-force is calculated for
 * each mass.
 * 
 * \param k  Stiffness of the spring
 * \param m  Mass of the objects (the same for all)
 * \param d  Damping factor
 * \param L  Length of the spring in rest-mode
 * \param dt Time-step
 * \param p1 Position of point 1 at the end of time-frame
 * \param v1 Velocity of point 1 at the end of time-frame
 * \param p2 Position of point 2 at the end of time-frame
 * \param v2 Velocity of point 2 at the end of time-frame
 * \param p3 Position of point 3 at the end of time-frame
 * \param v3 Velocity of point 3 at the end of time-frame
 */
void AdvanceTimeStep3(double k, double m, double d, double L, double dt,
                      Vec2& p1, Vec2& v1, Vec2& p2, Vec2& v2, Vec2& p3, Vec2& v3) {
    symplecticEuler(k, m, d, L, dt, p1, v1, p2, v2, p3, v3);
}
