//=============================================================================
//  Physically-based Simulation in Computer Graphics
//  ETH Zurich
//=============================================================================

#pragma once

#include <cmath>
#include <iostream>
#include "util.h"
#include "gauss_seidel.h"
#include "Utilities/Array2T.h"

#if defined(__APPLE__)
#include <GLUT/glut.h>
#else
#include "GL/glut.h"
#endif  

// resolution of the simulation
const int simulationResolution = 200;

// global control defines
#define OUTPATH "./output/"

const double timestep = 0.005;
// accuracy for gauss seidel solver
const double solverAccuracy = 1e-2;
// max. number of iterations
const int solverIterations = 1000;

// solver class
class Fluid2D
{
public:
	Fluid2D(int xRes, int yRes);

	// destructor
	~Fluid2D();

	// accessors 
	Array2d& density() { return _density; };
	Array2d& xVelocity() { return _xVelocity; };
	Array2d& yVelocity() { return _yVelocity; };
	Array2d& xForce() { return _xForce; };
	Array2d& yForce() { return _yForce; };
	Array2d& pressure() { return _pressure; };
	Array2d& divergence() { return _divergence; };
	int xRes() { return _xRes; };
	int yRes() { return _yRes; };
	double dt() { return _dt; };

	// Advance the simulation
	void step();

	// reset everything
	void reset();

	// add a density region 
	void addDensity(double xMin, double xMax, double yMin, double yMax);

public:
	bool _pngOutput;
	int _pngOutputInterval;

	bool _hasWind;

protected:
	int _xRes;                  // x resolution of simulation
	int _yRes;                  // y resolution of simulation
	double _dt;                 // simulation timestep
	double _impulseStrength;    // strength of addImpulse(), used for debugging purposes
	int _totalSteps;            // total timesteps taken so far

	Array2d _density;           // current density field
	Array2d _densityTemp;       // previous density field
	Array2d _pressure;          // current pressure field - x in the Poisson solver
	Array2d _xVelocity;         // current x velocity
	Array2d _yVelocity;         // current y velocity
	Array2d _xVelocityTemp;     // previous x velocity
	Array2d _yVelocityTemp;     // previous y velocity
	Array2d _xForce;            // current x force
	Array2d _yForce;            // current y force
	Array2d _divergence;        // velocity divergence - b in the Poisson solver
	SolverGaussSeidel* _solver; // whatever Poisson solver you chose

	// solve the Poisson equation for the pressure correction
	void solvePressure();

	// add the x and y forces into the velocity field
	void addForce();

	// add buoyancy into the velocity
	void addBuoyancy();

	// add wind into the velocity
	void addWind();

	// helper functions

	// compute divergence
	void computeDivergence();
	// correct velocities
	void correctVelocities();
	// advect values
	void advectValues();

	// set neumann boundaries
	void setNeumannX(Array2d &field);
	void setNeumannY(Array2d &field);
	void setZeroX(Array2d &field);
	void setZeroY(Array2d &field);

	// copy out boundaries
	void copyBorder(Array2d &field);
};
