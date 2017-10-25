//=============================================================================
//  Physically-based Simulation in Computer Graphics
//  ETH Zurich
//
//  Author: Peter Kaufmann, Christian Schumacher
//=============================================================================

#include "SimpleFEMDefs.h"
#include "FEMElementTri.h"
#include "FEMMesh.h"

// Flag if the geometric-calculation (= true) should be taken
// or by calculating the derivates with the kronecker-delta (= false)
static const bool useGeometric = true;


/**
 * \brief TASK 3: Assemble the stiffness matrix
 * To compute the stiffness-matrix, the derivates with respect to each point
 * are calculated.
 * IMPORTANT: The values will be computed and added to the stiffness-matrix only
 * if the following condition holds: global[i] >= global[j] => only when the global
 * index of the node i is higher or equal than the global index of node j.
 * 
 * \param pMesh Mesh of the current triangle. (Holds the stiffness-matrix)
 */
void FEMElementTri::Assemble(FEMMesh *pMesh) const {

	// Calculate gradients for all three vertices
	std::vector<Vector2> gradients(3);

	for (int i = 0; i < 3; i++) {
		if (useGeometric) {
			computeSingleBasisDerivGlobalGeom(i, gradients[i], pMesh);
		} else {
			computeSingleBasisDerivGlobalLES(i, gradients[i], pMesh);
		}
	}

	// Calculate element area. We will use it to calcualte stiffness later
	double area;
	computeElementArea(pMesh, area);

	// Fill in the stiffness-matrix. (Only add the element if the global ids of the nodes are i >= j)
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; ++j) {
			int globalI = GetGlobalNodeForElementNode(i);
			int globalJ = GetGlobalNodeForElementNode(j);

			if (globalI >= globalJ) {
				pMesh->AddToStiffnessMatrix(globalI, globalJ, area * (gradients[i] | gradients[j]));
			}
		}
	}
}


/**
 * \brief TASK 2: Geometric construction of global basis function derivatives
 * For this task, the derivates are computed with the geometrical interpretation of the basis function.
 * The gradient for a node is always normal to the opposite edge and its length is the inverse of the triangles height.
 * 
 * \param nodeId    Id of the current node for which the derivates are computed.
 * \param basisDerivGlobal  Output-parameter: Holds the derivates with respect to x and y.
 * \param pMesh     Mesh of the current triangle.
 */
void FEMElementTri::computeSingleBasisDerivGlobalGeom(int nodeId, Vector2 &basisDerivGlobal, const FEMMesh *pMesh) const {

	// Get other indices of other two vertices
	int x1 = (nodeId + 1) % 3;
	int x2 = (nodeId + 2) % 3;

	// Get coordinates of all vertices
	Vector2 X0 = pMesh->GetNodePosition(GetGlobalNodeForElementNode(nodeId));
	Vector2 X1 = pMesh->GetNodePosition(GetGlobalNodeForElementNode(x1));
	Vector2 X2 = pMesh->GetNodePosition(GetGlobalNodeForElementNode(x2));

	// Get the opposite edge and find the normal
	Vector2 edge = X2 - X1;
	Vector2 normal(-edge.y(), edge.x());
	normal = normal.normalized();
	
	// Calculate area of the triangle for the next formula
	double area;
	computeElementArea(pMesh, area);

	// Area = 0.5 * edge * height
	double height = 2 * area / edge.length();

	// Sanity check
	if (height <= 0) {
		std::cout << edge << " " << X0 << "not good\n";
	}

	// Scale normal with the inverse of the triangles height
	basisDerivGlobal = normal / height;
}


/**
* \brief TASK 1: Implement basis function derivatives
* For this task, the linear equation system with the kronecker delta is used to compute
* the derivates.
*
* \param nodeId    Id of the current node for which the derivates are computed.
* \param basisDerivGlobal  Output-parameter: Holds the derivates with respect to x and y.
* \param pMesh     Mesh of the current triangle.
*/
void FEMElementTri::computeSingleBasisDerivGlobalLES(int nodeId, Vector2 &basisDerivGlobal, const FEMMesh *pMesh) const {
	
	// We set the Kronecker delta symbols. This is only one for nodeId
	Vector3 roh(0, 0, 0);
	roh[nodeId] = 1;

	// Matrix for the left-hand side
	Matrix3x3 A;

	for (int c = 0; c < 3; ++c) {
		// Get node coordinates
		int globalNode = GetGlobalNodeForElementNode(c);
		Vector2 position = pMesh->GetNodePosition(globalNode);

		// Set them accordingly
		A(c, 0) = position.x();
		A(c, 1) = position.y();
		A(c, 2) = 1;
	}

	// Calculate the coefficients a, b and c
	Matrix3x3 A_inv = A.inverse();
	Vector3 linear = A_inv * roh;

	// Partial derivatives in x and in y leave us with a and b respectively
	basisDerivGlobal = Vector2(linear.x(), linear.y());
}


/**
 * \brief Compute the area of the triangle.
 * 
 * \param pMesh		Mesh of the current triangle.
 * \param area      Output-parameter: Area of the triangle.
 */
void FEMElementTri::computeElementArea(const FEMMesh* pMesh, double& area) const {

	// Get coordinates of all vertices
	Vector2 X0 = pMesh->GetNodePosition(GetGlobalNodeForElementNode(0));
	Vector2 X1 = pMesh->GetNodePosition(GetGlobalNodeForElementNode(1));
	Vector2 X2 = pMesh->GetNodePosition(GetGlobalNodeForElementNode(2));

	// Compute area based on heron's formula
	// Step 1: Compute length of each edge
	double a = (X0 - X1).length();
	double b = (X1 - X2).length();
	double c = (X2 - X0).length();

	// Step 2: Compute semiperimeter of triangle
	double s = (a + b + c) / 2.0;

	// Step 3: Apply heron's formula
	area = sqrt(s*(s - a)*(s - b)*(s - c));
}


/**
 * \brief Evaluate the basis function derivates for a given point on the plane.
 * 
 * \param nodeId    Id of the current node for which the derivates are computed.
 * \param pMesh     Mesh of the current triangle.
 * \param x         x-coordinate of the point to evaluate the basis function
 * \param y         y-coordinate of the point to evaluate the basis function
 * 
 * \return Value of the basis function for given point p(x, y).
 */
double FEMElementTri::evalSingleBasisGlobalLES(int nodeId, const FEMMesh* pMesh, double x, double y) const {
	// We set the Kronecker delta symbols. This is only one for nodeId
	Vector3 roh(0, 0, 0);
	roh[nodeId] = 1;

	// Matrix for the left-hand side
	Matrix3x3 A;

	for (int c = 0; c < 3; ++c) {
		// Get node coordinates
		int globalNode = GetGlobalNodeForElementNode(c);
		Vector2 position = pMesh->GetNodePosition(globalNode);

		// Set them accordingly
		A(c, 0) = position.x();
		A(c, 1) = position.y();
		A(c, 2) = 1;
	}

	// Calculate the coefficients a, b and c
	Matrix3x3 A_inv = A.inverse();
	Vector3 linear = A_inv * roh;

	// Calculate the area of the triangle
	double area;
	computeElementArea(pMesh, area);
	
	// Evaluate the basis-function at given point and multply it with the area
	return area * (linear.x() * x + linear.y() * y + linear.z());
}
