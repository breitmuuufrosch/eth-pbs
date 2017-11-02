//=============================================================================
//  Physically-based Simulation in Computer Graphics
//  ETH Zurich
//=============================================================================

#include "gauss_seidel.h"
#include "fluid2d.h"
#include "Utilities/Array2T.h"
#include "Utilities/Vector2T.h"

int clamp(int _value, int _min, int _max)
{
	return max(_min, min(_max, _value));
}

// Problem 1
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
		double rij = 0.0;
		for (int y = 1; y < _yRes - 1; y++) {
			for (int x = 1; x < _xRes - 1; x++) {
				rij = (4.0 * _field(x, y) - _field(x - 1, y) - _field(x + 1, y) - _field(x, y - 1) - _field(x, y + 1)) / (dx * dx);
				residual += std::pow(_b(x, y) - rij, 2);
			}
		}

		residual = std::sqrt(residual) / gridCells;


		// For your debugging, and ours, please add these prints after every iteration
		if (it == _iterations - 1) {
			printf("Pressure solver: it=%d , res=%f \n", it, residual);
		}

		if (residual < _accuracy) {
			printf("Pressure solver: it=%d , res=%f, converged \n", it, residual);

			break;
		}
	}
}

// Problem 2
void ExCorrectVelocities(int _xRes, int _yRes, double _dt, const Array2d &_pressure, Array2d &_xVelocity, Array2d &_yVelocity) {
	double dx = 1.0 / _xRes;
	double dy = 1.0 / _yRes;
	double roh = 1.0;

	// Note: velocity u_{i+1/2} is practically stored at i+1
	for (int y = 0; y < _yRes - 1; ++y) {
		for (int x = 0; x < _xRes - 1; ++x) {
			_xVelocity(x + 1, y) = _xVelocity(x + 1, y) - _dt / roh * (_pressure(x + 1, y) - _pressure(x, y)) / dx;
			_yVelocity(x, y + 1) = _yVelocity(x, y + 1) - _dt / roh * (_pressure(x, y + 1) - _pressure(x, y)) / dy;
		}
	}
}

// Problem 3
void BilinearInterpolation(double dx, double dy, Vector2T<int> &q11Index, Vector2T<double> &p, Vector2T<int> &destinationIndex, Array2d &_array, Array2d &_arrayTemp) {
	//Vector2T<double> q12(q11.x(), q11.y() + dy);
	//Vector2T<double> q21(q11.x() + dx, q11.y());
	q11Index = Vector2T<int>(clamp(q11Index.x(), 2, std::floor(1 / dx) - 2), clamp(q11Index.y(), 2, std::floor(1 / dy) - 2));
	Vector2T<double> q11(dx * q11Index.x(), dy * q11Index.y());
	Vector2T<double> q22(q11.x() + dx, q11.y() + dy);

	double f_x_y1 = (q22.x() - p.x()) / (q22.x() - q11.x()) * _array(q11Index.x(), q11Index.y())     + (p.x() - q11.x()) / (q22.x() - q11.x()) * _array(q11Index.x() + 1, q11Index.y());
	double f_x_y2 = (q22.x() - p.x()) / (q22.x() - q11.x()) * _array(q11Index.x(), q11Index.y() + 1) + (p.x() - q11.x()) / (q22.x() - q11.x()) * _array(q11Index.x() + 1, q11Index.y() + 1);

	_arrayTemp(destinationIndex.x(), destinationIndex.y()) = (q22.y() - p.y()) / (q22.y() - q11.y()) * f_x_y1 + (p.y() - q11.y()) / (q22.y() - q11.y()) * f_x_y2;

}

void Clamp(Vector2T<double> &P, double dx, double dy) {
	P.x() = min(max(P.x(), dx), 1 - dx);
	P.y() = min(max(P.y(), dy), 1 - dy);
}

void ExAdvectWithSemiLagrange(int _xRes, int _yRes, double _dt, Array2d &_xVelocity, Array2d &_yVelocity, Array2d &_density, Array2d &_densityTemp, Array2d &_xVelocityTemp, Array2d &_yVelocityTemp) {

	double dx = (1.0 / _xRes);
	double dy = (1.0 / _yRes);

	// Note: velocity u_{i+1/2} is practically stored at i+1
	for (int y = 1; y < _yRes - 1; y++) {
		for (int x = 1; x < _xRes - 1; x++) {
			Vector2T<double> u_x((_xVelocity(x, y) + _xVelocity(x+1, y)) / 2.0, (_yVelocity(x, y) + _yVelocity(x, y + 1)) / 2.0);
			//Vector2T<double> u(_xVelocity(x, y), _yVelocity(x, y));
			Vector2T<double> p((x) * dx, y * dy);
			Vector2T<double> Xp_x = p - _dt * u_x;
			Vector2T<double> X_density = Xp_x;
			Clamp(Xp_x, 2.5*dx, 2.5*dy);
			Clamp(X_density, 2.*dx, 2.*dy);

			int index_x0 = std::floor(Xp_x.x() * _xRes);
			int index_y0 = std::floor(Xp_x.y() * _yRes);

			Vector2T<int> q11(index_x0, index_y0);
			Vector2T<int> destinationIndex(x, y);

			BilinearInterpolation(dx, dy, q11, Xp_x, destinationIndex, _xVelocity, _xVelocityTemp);
			
			BilinearInterpolation(dx, dy, q11, Xp_x, destinationIndex, _yVelocity, _yVelocityTemp);
			
			BilinearInterpolation(dx, dy, q11, X_density, destinationIndex, _density, _densityTemp);
		}
	}

	// copy results from tmp buffers
	_density = _densityTemp;
	_xVelocity = _xVelocityTemp;
	_yVelocity = _yVelocityTemp;
}