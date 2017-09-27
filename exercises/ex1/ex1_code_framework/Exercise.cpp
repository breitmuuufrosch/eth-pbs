#include "Vec2.h"

// gravitational acceleration (9.81)
static const double g = 9.81;

// get the external force
double Fext(double m) {
    return m * g;
}

// get the internal force
double Fint(double k, double x, double L) {
    return k * (x - L);
}


void explicitEuler(double k, double m, double d, double L, double dt, double p1, double v1, double& p2, double& v2) {
    p2 = p2 + dt * v2;
    v2 = v2 + (1 / m) * (- Fint(k, p2, L) + Fext(m) - d * v2);

}

// Exercise 1
// hanging mass point
// Program the updates per-time-step of the position p2 and velocity
// v2 of the bottom point for the following integration methods:
// explicit Euler (method = 1) 
// symplectic Euler (method = 2) 
// explicit midpoint (method = 3)
// semi-implicit Euler (method = 4)

void AdvanceTimeStep1(double k, double m, double d, double L, double dt, int method, double p1, double v1, double& p2, double& v2)
{
    switch(method) {
        case 1 : 
            explicitEuler(k, m, d, L, dt, p1, v1, p2, v2);
            break;       // and exits the switch
        case 2 : 
            break;
        case 3 : 
            break;
        case 4 :
            break;
        default: 
            break;
    }

}


// Exercise 3
// falling triangle
void AdvanceTimeStep3(double k, double m, double d, double L, double dt,
                      Vec2& p1, Vec2& v1, Vec2& p2, Vec2& v2, Vec2& p3, Vec2& v3)
{
	p1 += Vec2(1,1);
}
