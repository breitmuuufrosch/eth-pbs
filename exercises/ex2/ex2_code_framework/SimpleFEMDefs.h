//=============================================================================
//  Physically-based Simulation in Computer Graphics
//  ETH Zurich
//
//  Author: Peter Kaufmann, Christian Schumacher
//=============================================================================

#include <assert.h>
#include <vector>
#include <map>
#include <math.h>

#include "Utilities/Vector2T.h"
#include "Utilities/Vector3T.h"
typedef Vector2T<double> Vector2;
typedef Vector3T<double> Vector3;

#include "Utilities/Matrix2x2T.h"
#include "Utilities/Matrix3x3T.h"
typedef Matrix2x2T<double> Matrix2x2;
typedef Matrix3x3T<double> Matrix3x3;

#include "Utilities/SparseSymmetricDynamicRowMatrixT.h"
#include "Utilities/SparseLinSolverPCGT.h"
typedef SparseSymmetricDynamicRowMatrixT<double> SparseSymmetricDynamicRowMatrix;
