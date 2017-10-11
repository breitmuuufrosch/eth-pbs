//=============================================================================
//  Physically-based Simulation in Computer Graphics
//  ETH Zurich
//
//  Author: Peter Kaufmann, Christian Schumacher
//=============================================================================

#pragma once

#include <iostream>
#include <string>
#include <cstdio>
#include <assert.h>

// Diagonally preconditioned conjugate gradient solver
template<class T>
class SparseLinSolverPCGT
{
public:
	/*! Solves the linear system 'matA * x = b' for the unknown vector \c x. \c matA must be symmetric, positive-definite.
		\c residual defines the desired accuracy of the solution and \c maxIterations is the maximum number of
		iterations to perform or -1 for an infinite amount of iterations. */
	bool SolveLinearSystem(
		SparseSymmetricDynamicRowMatrixT<T> &matA, std::vector<T> &x, const std::vector<T> &b,
		T residual, int maxIterations) {

		int n = matA.GetNumRows();

		if (x.size() != n || b.size() != n)
		{
			std::cerr << "SolveLinearSystem x and b dont match! " << x.size() << " " << b.size() << " n=" << n << " \n";
			assert(false);	// Dimension mismatch!
			return false;
		}

		std::vector<T> precond(n);
		std::vector<T> r(n);
		std::vector<T> d(n);
		std::vector<T> q(n);
		std::vector<T> s(n);

		// Prepare preconditioner:
		for (int i = 0; i < n; i++)
			precond[i] = 1 / matA(i, i);

		// From: Jonathan Richard Shewchuk, "An Introduction to the Conjugate Gradient Method
		// Without the Agonizing Pain", Edition 1 1/4, August 4, 1994
		// http://www.cs.cmu.edu/~quake-papers/painless-conjugate-gradient.pdf:

		// r = b - A*x
		matA.MultVector(x, r);
		for (int i = 0; i < n; i++)
			r[i] = b[i] - r[i];

		// d = M^-1*r
		for (int i = 0; i < n; i++)
			d[i] = precond[i] * r[i];

		// deltaNew = r^T*d
		T deltaNew = dotProd(r, d);

		// delta0 = deltaNew
		// Note: Using 1.0 here
		// to interpret residual as absolute error
		T delta0 = 1.0; //deltaNew;

		int iter = 0;
		while (maxIterations == -1 || iter < maxIterations)
		{
			if (deltaNew <= residual*residual*delta0)
				break;

			// q = A*d
			matA.MultVector(d, q);

			// alpha = deltaNew / (d^T*q)
			T alpha = deltaNew / dotProd(d, q);

			// x += alpha*d
			for (int i = 0; i < n; i++)
				x[i] += alpha*d[i];

			// NOTE: if i%50 == 0: r = b - A*x

			// r -= alpha*q;
			for (int i = 0; i < n; i++)
				r[i] -= alpha*q[i];

			// s = M^-1*r
			for (int i = 0; i < n; i++)
				s[i] = precond[i] * r[i];

			// deltaOld = deltaNew
			T deltaOld = deltaNew;

			// deltaNew = r^T*s
			deltaNew = dotProd(r, s);

			// beta = deltaNew / deltaOld
			T beta = deltaNew / deltaOld;

			// d = s + beta*d:
			for (int i = 0; i < n; i++)
				d[i] = s[i] + beta*d[i];

			iter++;
			std::cout << "PCG, iter=" << iter << ", deltaNew=" << sqrt(deltaNew) << " vs " << (residual) << "\n";
		}

		return true;
	}

private:
	// Computes the dot product of two vectors
	static T dotProd(const std::vector<T> &a, const std::vector<T> &b) {
		T v = 0;

		assert(a.size() == b.size());

		for (int i = 0; i < (int)a.size(); i++)
			v += a[i] * b[i];

		return v;
	}
};
