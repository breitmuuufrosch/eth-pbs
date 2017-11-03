//=============================================================================
//  Physically-based Simulation in Computer Graphics
//  ETH Zurich
//=============================================================================

#include <cmath>
#include "Utilities/Array2T.h"
#include "Utilities/Vector2T.h"
#include "Utilities/Matrix2x2T.h"


/**
 * \brief Clamping a value between an lower- and upper-bound, so that "lower <= value <= upper".
 * 
 * \param _value    Value to be clamped
 * \param _min      Lower-bound
 * \param _max      Upper-bound
 */
template<typename T>
T Clamp(T _value, T _min, T _max) {
	_value = (_min > _value) ? _min : _value;
	_value = (_max < _value) ? _max : _value;
	return _value;
}

/**
 * \brief Safe access on array. Clamp the invalid index to a valid index "0 <= i < size".
 * 
 * \param _array    Accessed array
 * \param x         X-index
 * \param y         Y-index
 * 
 * \return Value at the index which is projected onto the valid space.
 */
double Safe(Array2d &_array, int x, int y) {
	x = Clamp<int>(x, 0, _array.size(0) - 1);
	y = Clamp<int>(y, 0, _array.size(1) - 1);

	return _array(x, y);
}

/**
 * \brief Clamp a point within [0, 1]^2 to [dx, 1-dx]*[dy, 1-dy].
 * 
 * \param P	    In- and Output parameter:
 *              Input:  Point in [0, 1]^2 which needs to be clamped
 *              Output: Projected point to allowed area
 * \param dx    Margin on x-axis
 * \param dy    Margin on y-axis
 */
void Clamp(Vector2T<double> &P, double dx, double dy) {
	P.x() = Clamp<double>(P.x(), dx, 1 - dx);
	P.y() = Clamp<double>(P.y(), dy, 1 - dy);
}


/**
 * \brief Problem 1: Solve Poisson equation
 * Solves the poisson equation based on the formula given on the exercise-slides.
 * The residual is calculated to verify the convergence of the solution to stop earlier if possible.
 * 
 * \param _xRes
 *      Grid-resolution on x-axis.
 * \param _yRes
 *      Grid-resolution on y-axis.
 * \param _iterations
 *      Maximal number of iterations in case of no convergence.
 * \param _accuracy
 *      Convergence-criteria: as soon as the residual is less than the accuracy, the solution converged.
 * \param _field
 *      The field of units to solve the poisson equation for.
 * \param _b
 *      The divergence (right hand side) for the poisson-equation.
 */
void ExSolvePoisson(int _xRes, int _yRes, int _iterations, double _accuracy, Array2d &_field, Array2d &_b) {
	double dx = 1.0 / _xRes;
	int gridCells = (_xRes - 2) * (_yRes - 2);

	for (int it = 0; it < _iterations; ++it) {
		double residual = 0.0;

		// Note that the boundaries are handles by the framework, so you iterations should be similar to:
		for (int y = 1; y < _yRes - 1; y++) {
			for (int x = 1; x < _xRes - 1; x++) {
				// Todo: dx??
				_field(x, y) = (dx * dx * _b(x, y) + _field(x - 1, y) + _field(x, y - 1) + _field(x + 1, y) + _field(x, y + 1)) * 0.25;
			}
		}
		for (int y = 1; y < _yRes - 1; y++) {
			for (int x = 1; x < _xRes - 1; x++) {
				double rij = (4.0 * _field(x, y) - _field(x - 1, y) - _field(x + 1, y) - _field(x, y - 1) - _field(x, y + 1)) / (dx * dx);
				residual += std::pow(_b(x, y) - rij, 2);
			}
		}

		residual = std::sqrt(residual) / gridCells;


		// For your debugging, and ours, please add these prints after every iteration
		if (it == _iterations - 1) {
			printf("Pressure solver: it=%d, res=%f \n", it, residual);
		}

		if (residual < _accuracy) {
			printf("Pressure solver: it=%d, res=%f, converged \n", it, residual);

			break;
		}
	}
}

/**
* \brief Problem 2: Correct the velocities
* Compute pressure gradient with central differences for each velocity component.
*
* \param _xRes
*      Grid-resolution on x-axis.
* \param _yRes
*      Grid-resolution on y-axis.
* \param _dt
*      Time difference.
* \param _pressure
*      Pressure-field which is used to extract the velocities for each axis.
* \param _xVelocity
*      Field which stores the velocities on x-axis.
* \param _yVelocity
*      Field which stores the velocities on y-axis.
*/
void ExCorrectVelocities(int _xRes, int _yRes, double _dt, const Array2d &_pressure, Array2d &_xVelocity, Array2d &_yVelocity) {
	double dx = 1.0 / _xRes;
	double dy = 1.0 / _yRes;
	double rho = 1.0;

	// Note: velocity u_{i+1/2} is practically stored at i+1
	for (int y = 0; y < _yRes - 1; ++y) {
		for (int x = 0; x < _xRes - 1; ++x) {
			_xVelocity(x + 1, y) = _xVelocity(x + 1, y) - _dt / rho * (_pressure(x + 1, y) - _pressure(x, y)) / dx;
			_yVelocity(x, y + 1) = _yVelocity(x, y + 1) - _dt / rho * (_pressure(x, y + 1) - _pressure(x, y)) / dy;
		}
	}
}

// Problem 3
/**
 * \brief Bilinear interpolation for a given point with the 4 closest known values.
 * 
 * \param dx
 *      Distance change on x-axis.
 * \param dy
 *      Distance change on y-axis.
 * \param q11_i
 *      The location of the known Q_11 point (lower left) in the array.
 * \param p
 *      The point to interpolate.
 * \param _array
 *      Array with the needed units to advect.
 */
double BilinearInterpolation(double dx, double dy, Vector2T<int> &q11_i, Vector2T<double> &p, Array2d &_array) {
	Vector2T<double> q11(dx * q11_i.x(), dy * q11_i.y());
	Vector2T<double> q22(q11.x() + dx, q11.y() + dy);
	
	// Todo: Personaly, I prefer the bilinear-interpolation as matrix-multiplication. If you agree we can keep it, otherwise the other code is still there below.
	// The error between the two methods is around 1e-25, which leads visually to the same result.
	double factor = 1.0 / ((q22.x() - q11.x()) * (q22.y() - q11.y()));
	Vector2T<double> x_weights(q22.x() - p.x(), p.x() - q11.x());
	Vector2T<double> y_weights(q22.y() - p.y(), p.y() - q11.y());
	Matrix2x2T<double> f_values(Safe(_array, q11_i.x(), q11_i.y()), Safe(_array, q11_i.x(), q11_i.y() + 1), Safe(_array, q11_i.x() + 1, q11_i.y()), Safe(_array, q11_i.x() + 1, q11_i.y() + 1));

	return factor * x_weights | (f_values * y_weights);

	//double f_x_y1 = (q22.x() - p.x()) / (q22.x() - q11.x()) * Safe(_array, q11_i.x(), q11_i.y())
	//	+ (p.x() - q11.x()) / (q22.x() - q11.x()) * Safe(_array, q11_i.x() + 1, q11_i.y());
	//double f_x_y2 = (q22.x() - p.x()) / (q22.x() - q11.x()) * Safe(_array, q11_i.x(), q11_i.y() + 1)
	//	+ (p.x() - q11.x()) / (q22.x() - q11.x()) * Safe(_array, q11_i.x() + 1, q11_i.y() + 1);

	//return (q22.y() - p.y()) / (q22.y() - q11.y()) * f_x_y1 + (p.y() - q11.y()) / (q22.y() - q11.y()) * f_x_y2;
}


/**
 * \brief Problem 3: Perform semi-Lagrangian advection
 * Advect pressure and velocity and use bilinear interpolation.
 * 
 * \param _xRes
 *      Grid-resolution on x-axis.
 * \param _yRes
 *      Grid-resolution on y-axis.
 * \param _dt
 *      Time difference.
 * \param _density
 *      Output-parameter: Field which holds the density after semi-Lagrangian advection.
 * \param _xVelocity
 *      Output-parameter: Field which holds the x-velocity after semi-Lagrangian advection.
 * \param _yVelocity
 *      Output-parameter: Field which holds the y-velocity after semi-Lagrangian advection.
 * \param _densityTemp
 *      Buffer-field which can be used during computation for density.
 * \param _xVelocity
 *      Buffer-field which can be used during computation for x-velocity.
 * \param _yVelocity
 *      Buffer-field which can be used during computation for y-velocity.
 */
void ExAdvectWithSemiLagrange(int _xRes, int _yRes, double _dt, Array2d &_xVelocity, Array2d &_yVelocity, Array2d &_density, Array2d &_densityTemp, Array2d &_xVelocityTemp, Array2d &_yVelocityTemp) {
	double dx = 1.0 / _xRes;
	double dy = 1.0 / _yRes;

	// Note: velocity u_{i+1/2} is practically stored at i+1
	for (int y = 1; y < _yRes - 1; y++) {
		for (int x = 1; x < _xRes - 1; x++) {
			Vector2T<double> u_x((_xVelocity(x, y) + _xVelocity(x+1, y)) / 2.0, (_yVelocity(x, y) + _yVelocity(x, y + 1)) / 2.0);
			Vector2T<double> p(x * dx, y * dy);
			Vector2T<double> Xp_velocity = p - _dt * u_x;
			Vector2T<double> Xp_density = Xp_velocity;
			
			//Todo: Please re-check.. when clamping is active, the pillar thends to the left side after some time..
			Clamp(Xp_velocity, 2.5*dx, 2.5*dy);
			Clamp(Xp_density, 2.0*dx, 2.0*dy);

			int indexV_x0 = std::floor(Xp_velocity.x() * _xRes);
			int indexV_y0 = std::floor(Xp_velocity.y() * _yRes);
			Vector2T<int> q11_velocity(indexV_x0, indexV_y0);

			int indexD_x0 = std::floor(Xp_density.x() * _xRes);
			int indexD_y0 = std::floor(Xp_density.y() * _yRes);
			Vector2T<int> q11_density(indexD_x0, indexD_y0);

			_xVelocityTemp(x, y) = BilinearInterpolation(dx, dy, q11_velocity, Xp_velocity, _xVelocity);
			_yVelocityTemp(x, y) = BilinearInterpolation(dx, dy, q11_velocity, Xp_velocity, _yVelocity);
			_densityTemp(x, y)   = BilinearInterpolation(dx, dy, q11_density, Xp_density, _density);
		}
	}

	// Copy results from tmp buffers. (do not copy the boarders since we did not update them)
	for (int y = 1; y < _yRes - 1; y++) {
		for (int x = 1; x < _xRes - 1; x++) {
			_density(x, y)   = _densityTemp(x, y);
			_xVelocity(x, y) = _xVelocityTemp(x, y);
			_yVelocity(x, y) = _yVelocityTemp(x, y);
		}
	}
}