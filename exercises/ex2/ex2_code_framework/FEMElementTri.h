//=============================================================================
//  Physically-based Simulation in Computer Graphics
//  ETH Zurich
//
//  Author: Peter Kaufmann, Christian Schumacher
//=============================================================================

#pragma once

class FEMMesh;

class FEMElementTri
{
public:
	FEMElementTri(int node0, int node1, int node2)
	{
		m_nodes[0] = node0;
		m_nodes[1] = node1;
		m_nodes[2] = node2;
	}

	// Adds the contributions of this element to the global stiffness matrix
	void Assemble(FEMMesh *pMesh) const;

	// Returns the number of nodes in the element
	int GetNumElementNodes() const { return 3; }

	// Returns the global ID of the element node el
	int GetGlobalNodeForElementNode(int el) const { return m_nodes[el]; }

//private:

	// Computes for a basis function its x/y derivatives geometrically
	void computeSingleBasisDerivGlobalGeom(int nodeId, Vector2 &basisDerivGlobal, const FEMMesh *pMesh) const;

	// Computes for a basis function its x/y derivatives geometrically
	void computeSingleBasisDerivGlobalLES(int nodeId, Vector2 &basisDerivGlobal, const FEMMesh *pMesh) const;
	double evalSingleBasisGlobalLES(int nodeId, const FEMMesh *pMesh, double x, double y) const;

	void computeElementArea(const FEMMesh *pMesh, double &area) const;
	void setPositions(Vector2 *pos, const FEMMesh *pMesh) const;

private:
	// The global IDs of the three nodes of the triangle
	int m_nodes[3];
};
