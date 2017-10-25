//=============================================================================
//  Physically-based Simulation in Computer Graphics
//  ETH Zurich
//=============================================================================

#include "util.h"
#include "fluid2d.h"
#include <string>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include "Utilities/Array2T.h"

#if !defined(_WIN32)
#include <sys/time.h>
#endif

Fluid2D::Fluid2D(int xRes, int yRes)
:	_xRes(xRes),
	_yRes(yRes),
	_impulseStrength(5000.0f),
	_pngOutput(false),
	_pngOutputInterval(2),
	_hasWind(false)
{
	// adapt timestep to acceleration
	_dt = timestep * sqrt((xRes + yRes) * 0.5);
	_totalSteps = 0;

	// allocate solver
	_solver = new SolverGaussSeidel(solverIterations, _dt, solverAccuracy);

	// allocate arrays
	_divergence.resize(_xRes, _yRes);
	_pressure.resize(_xRes, _yRes);
	_xVelocity.resize(_xRes + 1, _yRes);
	_yVelocity.resize(_xRes, _yRes + 1);
	_xVelocityTemp.resize(_xRes + 1, _yRes);
	_yVelocityTemp.resize(_xRes, _yRes + 1);
	_xForce.resize(_xRes + 1, _yRes);
	_yForce.resize(_xRes, _yRes + 1);
	_density.resize(_xRes, _yRes);
	_densityTemp.resize(_xRes, _yRes);

	std::cout << " timestep = " << _dt << "\n";

	// init arrays
	reset();
}

Fluid2D::~Fluid2D()
{
	if (_solver)
		delete _solver;
}

// step simulation once
void Fluid2D::step()
{
	long int startt = getTime();

	// add in new forces
	addBuoyancy();
	if (_hasWind)
		addWind();
	addForce();

	// remove divergence
	solvePressure();

	// advect everything
	advectValues();

	if (_pngOutput)
	{
		// dump original densities
		std::string prefix = std::string(OUTPATH "/density.");
		if (_totalSteps % _pngOutputInterval == 0)
			dumpNumberedImage(_totalSteps, prefix, _density);
	}

	// reset forces
	_xForce.zero();
	_yForce.zero();

	_totalSteps++;

	long int stopt = getTime();
	std::cout << "Step " << _totalSteps << " took " << timeToString(stopt - startt) << "\n";
}

void Fluid2D::addForce()
{
	for (int i = 0; i < _xVelocity.size(0); i++)
	{
		for (int j = 0; j < _xVelocity.size(1); j++)
		{
			_xVelocity(i, j) += _dt * _xForce(i, j);
		}
	}

	for (int i = 0; i < _yVelocity.size(0); i++)
	{
		for (int j = 0; j < _yVelocity.size(1); j++)
		{
			_yVelocity(i, j) += _dt * _yForce(i, j);
		}
	}
}

void Fluid2D::reset()
{
	_density.zero();
	_densityTemp.zero();
	_divergence.zero();
	_pressure.zero();
	_xVelocity.zero();
	_yVelocity.zero();
	_xVelocityTemp.zero();
	_yVelocityTemp.zero();
	_xForce.zero();
	_yForce.zero();
	_totalSteps = 0;
}

void Fluid2D::solvePressure()
{
	// copy out the boundaries 
	setNeumannX(_xVelocity);
	setNeumannY(_yVelocity);
	setZeroY(_xVelocity);
	setZeroX(_yVelocity);

	computeDivergence();

	// solve Poisson equation
	copyBorder(_pressure);
	_solver->solvePoisson(_xRes, _yRes, _pressure, _divergence);

	correctVelocities();
}

void Fluid2D::addBuoyancy()
{
	double scaling = 64.0f / (double)_xRes;

	// add buoyancy
	for (int i = 0; i < _yForce.size(0); i++)
	{
		for (int j = 1; j < _yForce.size(1) - 1; j++)
		{
			_yForce(i, j) += 0.1 * (_density(i, j - 1) + _density(i, j)) / 2.0 * scaling;
		}
	}
}

void Fluid2D::addWind()
{
	double scaling = 64.0f / (double)_xRes;

	static double r = 0.0;
	r += 1;

	const double fx = 2e-2 * cos(5e-2 * r) * cos(3e-2 * r) * scaling;

	// add wind
	for (int i = 0; i < _xForce.size(0); i++)
	{
		for (int j = 0; j < _xForce.size(1); j++)
		{
			_xForce(i, j) += fx;
		}
	}
}

void Fluid2D::addDensity(double xMin, double xMax, double yMin, double yMax)
{
	// add fluid square 
	for (int y = (int)(yMin * _yRes); y < (int)(yMax * _yRes); y++)
	{
		for (int x = (int)(xMin * _xRes); x < (int)(xMax * _xRes); x++)
		{
			_density(x, y) = 1.0;
		}
	}
}

void Fluid2D::computeDivergence()
{
	// calculate divergence
	const double dx = 1.0 / _xRes;
	const double idtx = 1.0 / (_dt * dx);

	double absDiv = 0.;
	for (int y = 1; y < _yRes - 1; y++)
	{
		for (int x = 1; x < _xRes - 1; x++)
		{
			const double xComponent = (_xVelocity(x + 1, y) - _xVelocity(x, y)) * idtx;
			const double yComponent = (_yVelocity(x, y + 1) - _yVelocity(x, y)) * idtx;
			_divergence(x, y) = -(xComponent + yComponent);
		}
	}

	/*for (int i = 0; i < _xRes*_yRes; i++)
	{
		if (fabs(_divergence[i]) > 1e-6)
			printf("%d %e\n", i, _divergence[i]);
	}*/
}

void Fluid2D::copyBorder(Array2d &field)
{
	const int sx = field.size(0);
	const int sy = field.size(1);
	for (int y = 0; y < sy; y++)
	{
		field(0, y) = field(1, y);
		field(sx - 1, y) = field(sx - 2, y);
	}
	for (int x = 0; x < sx; x++)
	{
		field(x, 0) = field(x, 1);
		field(x, sy - 1) = field(x, sy - 2);
	}
}

void Fluid2D::setNeumannX(Array2d &field)
{
	const int sx = field.size(0);
	const int sy = field.size(1);
	for (int y = 0; y < sy; y++)
	{
		field(0, y) = field(2, y);
		field(sx - 1, y) = field(sx - 3, y);
	}
}

void Fluid2D::setNeumannY(Array2d &field)
{
	const int sx = field.size(0);
	const int sy = field.size(1);
	for (int x = 0; x < sx; x++)
	{
		field(x, 0) = field(x, 2);
		field(x, sy - 1) = field(x, sy - 3);
	}
}

void Fluid2D::setZeroX(Array2d &field)
{
	const int sx = field.size(0);
	const int sy = field.size(1);
	for (int y = 0; y < sy; y++)
	{
		field(0, y) = 0.0;
		field(sx - 1, y) = 0.0;
	}
}

void Fluid2D::setZeroY(Array2d &field)
{
	const int sx = field.size(0);
	const int sy = field.size(1);
	for (int x = 0; x < sx; x++)
	{
		field(x, 0) = 0.0;
		field(x, sy - 1) = 0.0;
	}
}

// call exercises
extern void ExSolvePoisson(int xRes, int yRes, int _iterations, double _accuracy, Array2d &field, Array2d &b);
extern void ExCorrectVelocities(int _xRes, int _yRes, double _dt, const Array2d &_pressure, Array2d &_xVelocity, Array2d &_yVelocity);
extern void ExAdvectWithSemiLagrange(int _xRes, int _yRes, double _dt, Array2d &_xVelocity, Array2d &_yVelocity, Array2d &_density, Array2d &_densityTemp, Array2d &_xVelocityTemp, Array2d &_yVelocityTemp);

void SolverGaussSeidel::solvePoisson(int xRes, int yRes, Array2d &field, Array2d &b)
{
	ExSolvePoisson(xRes, yRes, _iterations, _accuracy, field, b);
}

void Fluid2D::correctVelocities()
{
	ExCorrectVelocities(_xRes, _yRes, _dt, _pressure, _xVelocity, _yVelocity);
}

void Fluid2D::advectValues()
{
	ExAdvectWithSemiLagrange(_xRes, _yRes, _dt, _xVelocity, _yVelocity, _density, _densityTemp, _xVelocityTemp, _yVelocityTemp);
}
