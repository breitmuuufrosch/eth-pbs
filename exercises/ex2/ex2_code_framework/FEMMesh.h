//=============================================================================
//  Physically-based Simulation in Computer Graphics
//  ETH Zurich
//
//  Author: Peter Kaufmann, Christian Schumacher
//=============================================================================

#pragma once

#include "FEMElementTri.h"

// Nodal boundary condition (Dirichlet boundary condition)
class BoundaryCondition
{
public:
	BoundaryCondition(int nodeID, double val)
	{
		m_nodeID = nodeID;
		m_val = val;
	}

	// Gets the global ID of the node at which the boundary condition applies
	int GetID() const { return m_nodeID; }

	// Gets the value of the Dirichlet boundary condition
	double GetValue() const { return m_val; }

private:
	// Node at which the boundary condition applies
	int m_nodeID;

	// Value of the Dirichlet boundary condition
	double m_val;
};

// A mesh consisting of nodes and elements
class FEMMesh {
public:
	// Adds a node to the mesh and returns the ID of the node
	int AddNode(const Vector2 &position)
	{
		m_nodes.push_back(position);
		return m_nodes.size() - 1;
	}

	// Adds an element to the mesh and returns the ID of the element
	int AddElement(FEMElementTri element)
	{
		m_elements.push_back(element);
		return m_elements.size() - 1;
	}

	// Sets the right-hand side of the problem
	void SetRHS(const std::vector<double> &rhs) { m_rhs = rhs; }

	// Returns the number of nodes (= degrees of freedom) in the mesh
	int GetNumNodes() const
	{
		return m_nodes.size();
	}

	// Returns the number of elements in the mesh
	int GetNumElements() const
	{
		return m_elements.size();
	}

	// Returns the element with ID \c elID
	const FEMElementTri &GetElement(int elID) const
	{
		return m_elements[elID];
	}

	// From IFEMAssembler:
	virtual const Vector2 &GetNodePosition(int nodeID) const
	{
		return m_nodes[nodeID];
	}

	// From IFEMAssembler:
	virtual void AddToStiffnessMatrix(int u, int v, double val)
	{
		m_matK(u, v) += val;
	}

	// access Stiffness matrix
	SparseSymmetricDynamicRowMatrix& getMat() { return m_matK; }


	// Solves the problem. NOTE: Call \c BuildStiffnessMatrix() before calling this
	bool Solve(std::vector<double> &solution, const std::vector<BoundaryCondition> &boundaryConds) const
	{
		if (m_matK.GetNumRows() != m_rhs.size())
		{
			std::cerr << "FEMMesh::Solve matrix and RHS dont match! " << m_matK.GetNumRows() << " vs " << m_rhs.size() << " \n";
			return false;	// Dimension mismatch! RHS not computed?
		}

		solution.resize(m_matK.GetNumRows());

		// Create copies of K and RHS:
		std::vector<double> rhs = m_rhs;
		SparseSymmetricDynamicRowMatrix matK = m_matK;

		// Apply Dirichlet boundary conditions by modifying K and RHS:
		for (const BoundaryCondition &bc : boundaryConds)
		{
			matK.FixSolution(rhs, bc.GetID(), bc.GetValue());
		}

		// Solve the problem:
		SparseLinSolverPCGT<double> solver;
		return solver.SolveLinearSystem(matK, solution, rhs, (double)1e-6, 1000);
	}

	void printStiffnessMatrix() const
	{
		int n = m_matK.GetNumRows();
		for (int i = 0; i < n; i++)
		{
			printf("[");
			for (int j = 0; j < n; j++)
			{
				double d = (i > j) ? m_matK(i, j) : m_matK(j, i);
				printf("%f ", d);
			}
			printf("]\n");
		}
	}

private:
	// Position of all nodes in the mesh
	std::vector<Vector2> m_nodes;

	// All elements in the mesh
	std::vector<FEMElementTri> m_elements;

	// Stiffness matrix
	SparseSymmetricDynamicRowMatrix m_matK;

	// Current right-hand side
	std::vector<double> m_rhs;
};
