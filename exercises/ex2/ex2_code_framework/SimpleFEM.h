//=============================================================================
//  Physically-based Simulation in Computer Graphics
//  ETH Zurich
//
//  Author: Peter Kaufmann, Christian Schumacher
//=============================================================================

#pragma once

#include "FEMMesh.h"

// Class containing various helper functions to test the FEMMesh class
class SimpleFEM
{
public:
	/*! Creates a uniform FEM triangle mesh with \c nodesX * \c nodesY nodes for the
		domain [0,0] - [1,1]. */
	static FEMMesh CreateUniformGridMesh(int nodesX, int nodesY);

	/*! Computes the boundary conditions for a known solution on the domain [0,0] - [1,1]. */
	static void ComputeBoundaryConditions(const FEMMesh &mesh, std::vector<BoundaryCondition> &boundaryConditions);
	/*! Computes the right-hand side for a known solution on the domain [0,0] - [1,1]. */
	static void ComputeRHS(const FEMMesh &mesh, std::vector<double> &rhs);
	/*! Computes the error of a numerical solution and its energy norm */
	static void computeError(FEMMesh &mesh,  const std::vector<double> &sol_num, std::vector<double> &verror, double& err_nrm);

	// Returns \c true iff the point \c pos is on the boundary of the domain [0,0] - [1,1]
	static bool isOnBoundary(const Vector2 &pos);
};
