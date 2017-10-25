//=============================================================================
//  Physically-based Simulation in Computer Graphics
//  ETH Zurich
//=============================================================================

#include "gauss_seidel.h"
#include "fluid2d.h"
#include "Utilities/Array2T.h"

// Problem 1
void ExSolvePoisson(int _xRes, int _yRes, int _iterations, double _accuracy, Array2d &_field, Array2d &_b)
{
	double dx = 1.0 / _xRes;
	
	// Note that the boundaries are handles by the framework, so you iterations should be similar to:
	for (int y = 1; y < _yRes - 1; y++)
	{
		for (int x = 1; x < _xRes - 1; x++)
		{
			
		}
	}
	
	// For your debugging, and ours, please add these prints after every iteration
	//if(it == _iterations - 1) 
	//	printf("Pressure solver: it=%d , res=%f \n", it, residual);
	//if(residual < _accuracy)
	//	printf("Pressure solver: it=%d , res=%f, converged \n", it, residual);
}

// Problem 2
void ExCorrectVelocities(int _xRes, int _yRes, double _dt, const Array2d &_pressure, Array2d &_xVelocity, Array2d &_yVelocity)
{
	double dx = 1.0 / _xRes;

	// Note: velocity u_{i+1/2} is practically stored at i+1
}

// Problem 3
void ExAdvectWithSemiLagrange(int _xRes, int _yRes, double _dt, Array2d &_xVelocity, Array2d &_yVelocity, Array2d &_density, Array2d &_densityTemp, Array2d &_xVelocityTemp, Array2d &_yVelocityTemp)
{
	// Note: velocity u_{i+1/2} is practically stored at i+1
}
