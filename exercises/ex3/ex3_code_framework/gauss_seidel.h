//=============================================================================
//  Physically-based Simulation in Computer Graphics
//  ETH Zurich
//=============================================================================

#pragma once

#include "Utilities/Array2T.h"

class SolverGaussSeidel
{
public:
	// constructor
	// iterations   maximum number of iterations to run the solver
	// dt           timestep being used by the fluid simulator
	SolverGaussSeidel(int iterations, double dt, double accuracy) :
		_iterations(iterations), _dt(dt), _accuracy(accuracy) {};
	
	// Poisson solver
	// xRes     x resolution of pressure grid
	// yRes     y resolution of pressure grid
	// field    the pressure grid
	// b        right hand side of the Poisson problem
	void solvePoisson(int xRes, int yRes, Array2d &field, Array2d &b);

protected:
	int _iterations;  // maximum iterations to run the solver
	double _dt;       // timestep being used by the simulation
	double _accuracy; // how small should the residual be?
};
