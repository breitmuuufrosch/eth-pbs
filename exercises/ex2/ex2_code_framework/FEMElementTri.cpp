//=============================================================================
//  Physically-based Simulation in Computer Graphics
//  ETH Zurich
//
//  Author: Peter Kaufmann, Christian Schumacher
//=============================================================================

#include "SimpleFEMDefs.h"
#include "FEMElementTri.h"
#include "FEMMesh.h"

static const bool useGeometric = true;



// TASK 3
void FEMElementTri::Assemble(FEMMesh *pMesh) const
{
	std::vector<Vector2> gradients(3);

	for (int i = 0; i < 3; i++) {
		if (useGeometric) {
			computeSingleBasisDerivGlobalGeom(i, gradients[i], pMesh);
		} else {
			computeSingleBasisDerivGlobalLES(i, gradients[i], pMesh);
		}
	}

	double area;
	computeElementArea(pMesh, area);

	// TODO: @utesic Check order
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

// TASK 2
void FEMElementTri::computeSingleBasisDerivGlobalGeom(int nodeId, Vector2 &basisDerivGlobal, const FEMMesh *pMesh) const {
	int x1 = (nodeId + 1) % 3;
	int x2 = (nodeId + 2) % 3;

	Vector2 X0 = pMesh->GetNodePosition(GetGlobalNodeForElementNode(nodeId));
	Vector2 X1 = pMesh->GetNodePosition(GetGlobalNodeForElementNode(x1));
	Vector2 X2 = pMesh->GetNodePosition(GetGlobalNodeForElementNode(x2));

	Vector2 edge = X2 - X1;
	Vector2 normal(-edge.y(), edge.x());
	normal = normal.normalized();
	
	double area;
	computeElementArea(pMesh, area);

	double height = 2 * area / edge.length(); // abs((edge.x() * X0.y() - edge.y() * X0.x())) / edge.length();

	if (height == 0) {
		std::cout << edge << " " << X0 << "not good\n";
	}

	basisDerivGlobal = normal / height;
}

// TASK 1
void FEMElementTri::computeSingleBasisDerivGlobalLES(int nodeId, Vector2 &basisDerivGlobal, const FEMMesh *pMesh) const {
	Vector3 roh(0, 0, 0);
	roh[nodeId] = 1;

	std::vector<int> index({ 0, 1, 2 });
	//index.erase(remove(index.begin(), index.end(), nodeId), index.end());

	Matrix3x3 A;

	for (auto const& c : index) {
		int globalNode = GetGlobalNodeForElementNode(c);
		Vector2 position = pMesh->GetNodePosition(globalNode);

		A(c, 0) = position.x();
		A(c, 1) = position.y();
		A(c, 2) = 1;
	}

	Matrix3x3 A_inv = A.inverse();
	Vector3 linear = A_inv * roh;

	basisDerivGlobal = Vector2(linear.x(), linear.y());
}

void FEMElementTri::computeElementArea(const FEMMesh* pMesh, double& area) const
{
	Vector2 X0 = pMesh->GetNodePosition(GetGlobalNodeForElementNode(0));
	Vector2 X1 = pMesh->GetNodePosition(GetGlobalNodeForElementNode(1));
	Vector2 X2 = pMesh->GetNodePosition(GetGlobalNodeForElementNode(2));

	double a = (X0 - X1).length();
	double b = (X1 - X2).length();
	double c = (X2 - X0).length();

	double s = (a + b + c) / 2.0;

	area = sqrt(s*(s - a)*(s - b)*(s - c));
}

double FEMElementTri::evalSingleBasisGlobalLES(int nodeId, const FEMMesh* pMesh, double x, double y) const
{
	// TODO: refactor
	Vector3 roh(0, 0, 0);
	roh[nodeId] = 1;

	std::vector<int> index({ 0, 1, 2 });
	//index.erase(remove(index.begin(), index.end(), nodeId), index.end());

	Matrix3x3 A;

	for (auto const& c : index) {
		int globalNode = GetGlobalNodeForElementNode(c);
		Vector2 position = pMesh->GetNodePosition(globalNode);

		A(c, 0) = position.x();
		A(c, 1) = position.y();
		A(c, 2) = 1;
	}

	Matrix3x3 A_inv = A.inverse();
	Vector3 linear = A_inv * roh;

	double area;
	computeElementArea(pMesh, area);
	
	return area * (linear.x() * x + linear.y() * y + linear.z());
}
