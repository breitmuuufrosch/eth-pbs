//=============================================================================
//  Physically-based Simulation in Computer Graphics
//  ETH Zurich
//
//  Author: Peter Kaufmann, Christian Schumacher
//=============================================================================

#pragma once

#include <iostream>
#include <cstdio>
#include <limits>
#include <assert.h>
#include "Vector3T.h"

template<typename Scalar>
class Matrix3x3T
{
public:
	// Empty constructor - object remains uninitialized.
	Matrix3x3T() {}

	// Copy-Constructor
	Matrix3x3T(const Matrix3x3T &p) {
		m_elem[0][0] = p.m_elem[0][0];
		m_elem[0][1] = p.m_elem[0][1];
		m_elem[0][2] = p.m_elem[0][2];
		m_elem[1][0] = p.m_elem[1][0];
		m_elem[1][1] = p.m_elem[1][1];
		m_elem[1][2] = p.m_elem[1][2];
		m_elem[2][0] = p.m_elem[2][0];
		m_elem[2][1] = p.m_elem[2][1];
		m_elem[2][2] = p.m_elem[2][2];
	}

	Scalar& operator() (int _i, int _j) {
		return m_elem[_i][_j];
	}

	const Scalar& operator() (int _i, int _j) const {
		return m_elem[_i][_j];
	}

	// +=-Operator
	Matrix3x3T& operator+=(const Matrix3x3T& p) {
		m_elem[0][0] += p.m_elem[0][0];
		m_elem[0][1] += p.m_elem[0][1];
		m_elem[0][2] += p.m_elem[0][2];
		m_elem[1][0] += p.m_elem[1][0];
		m_elem[1][1] += p.m_elem[1][1];
		m_elem[1][2] += p.m_elem[1][2];
		m_elem[2][0] += p.m_elem[2][0];
		m_elem[2][1] += p.m_elem[2][1];
		m_elem[2][2] += p.m_elem[2][2];
		return *this;
	}

	// -=-Operator
	Matrix3x3T& operator-=(const Matrix3x3T& p) {
		m_elem[0][0] -= p.m_elem[0][0];
		m_elem[0][1] -= p.m_elem[0][1];
		m_elem[0][2] -= p.m_elem[0][2];
		m_elem[1][0] -= p.m_elem[1][0];
		m_elem[1][1] -= p.m_elem[1][1];
		m_elem[1][2] -= p.m_elem[1][2];
		m_elem[2][0] -= p.m_elem[2][0];
		m_elem[2][1] -= p.m_elem[2][1];
		m_elem[2][2] -= p.m_elem[2][2];
		return *this;
	}

	// /=-Operator
	Matrix3x3T& operator*=(Scalar s) {
		m_elem[0][0] *= s;
		m_elem[0][1] *= s;
		m_elem[0][2] *= s;
		m_elem[1][0] *= s;
		m_elem[1][1] *= s;
		m_elem[1][2] *= s;
		m_elem[2][0] *= s;
		m_elem[2][1] *= s;
		m_elem[2][2] *= s;
		return *this;
	}

	// *=-Operator : Matrix * Matrix
	Matrix3x3T& operator*=(const Matrix3x3T& p) {
		return (*this = *this * p);
	}

	// +-Operator
	Matrix3x3T  operator+(const Matrix3x3T& p) const {
		return Matrix3x3T(m_elem[0][0] + p.m_elem[0][0],
			m_elem[0][1] + p.m_elem[0][1],
			m_elem[0][2] + p.m_elem[0][2],
			m_elem[1][0] + p.m_elem[1][0],
			m_elem[1][1] + p.m_elem[1][1],
			m_elem[1][2] + p.m_elem[1][2],
			m_elem[2][0] + p.m_elem[2][0],
			m_elem[2][1] + p.m_elem[2][1],
			m_elem[2][2] + p.m_elem[2][2]);
	}

	// --Operator
	Matrix3x3T  operator-(const Matrix3x3T& p) const {
		return Matrix3x3T(m_elem[0][0] - p.m_elem[0][0],
			m_elem[0][1] - p.m_elem[0][1],
			m_elem[0][2] - p.m_elem[0][2],
			m_elem[1][0] - p.m_elem[1][0],
			m_elem[1][1] - p.m_elem[1][1],
			m_elem[1][2] - p.m_elem[1][2],
			m_elem[2][0] - p.m_elem[2][0],
			m_elem[2][1] - p.m_elem[2][1],
			m_elem[2][2] - p.m_elem[2][2]);
	}

	// -Operator
	Matrix3x3T  operator/(Scalar s) const {
		return Matrix3x3T(m_elem[0][0] / s, m_elem[0][1] / s, m_elem[0][2] / s,
			m_elem[1][0] / s, m_elem[1][1] / s, m_elem[1][2] / s,
			m_elem[2][0] / s, m_elem[2][1] / s, m_elem[2][2] / s);
	}

	// *-Operator : Matrix * Scalar
	Matrix3x3T  operator*(Scalar s) const {
		return Matrix3x3T(m_elem[0][0] * s, m_elem[0][1] * s, m_elem[0][2] * s,
			m_elem[1][0] * s, m_elem[1][1] * s, m_elem[1][2] * s,
			m_elem[2][0] * s, m_elem[2][1] * s, m_elem[2][2] * s);
	}

	// friend *-Operator : Scalar * Matrix
	friend Matrix3x3T operator*(Scalar s, const Matrix3x3T& p) {
		return Matrix3x3T(p.m_elem[0][0] * s, p.m_elem[0][1] * s, p.m_elem[0][2] * s,
			p.m_elem[1][0] * s, p.m_elem[1][1] * s, p.m_elem[1][2] * s,
			p.m_elem[2][0] * s, p.m_elem[2][1] * s, p.m_elem[2][2] * s);
	}

	// *-Operator : Matrix * Vector
	Vector3T<Scalar> operator*(const Vector3T<Scalar>& vec) const {
		return Vector3T<Scalar>(m_elem[0][0] * vec[0] +
			m_elem[0][1] * vec[1] +
			m_elem[0][2] * vec[2],
			m_elem[1][0] * vec[0] +
			m_elem[1][1] * vec[1] +
			m_elem[1][2] * vec[2],
			m_elem[2][0] * vec[0] +
			m_elem[2][1] * vec[1] +
			m_elem[2][2] * vec[2]);
	}

	// Computes ((*this)^T * vec)
	Matrix3x3T  operator*(const Matrix3x3T& p) const {
		Matrix3x3T result;
		result.m_elem[0][0] = (m_elem[0][0] * p.m_elem[0][0] +
			m_elem[0][1] * p.m_elem[1][0] +
			m_elem[0][2] * p.m_elem[2][0]);
		result.m_elem[0][1] = (m_elem[0][0] * p.m_elem[0][1] +
			m_elem[0][1] * p.m_elem[1][1] +
			m_elem[0][2] * p.m_elem[2][1]);
		result.m_elem[0][2] = (m_elem[0][0] * p.m_elem[0][2] +
			m_elem[0][1] * p.m_elem[1][2] +
			m_elem[0][2] * p.m_elem[2][2]);
		result.m_elem[1][0] = (m_elem[1][0] * p.m_elem[0][0] +
			m_elem[1][1] * p.m_elem[1][0] +
			m_elem[1][2] * p.m_elem[2][0]);
		result.m_elem[1][1] = (m_elem[1][0] * p.m_elem[0][1] +
			m_elem[1][1] * p.m_elem[1][1] +
			m_elem[1][2] * p.m_elem[2][1]);
		result.m_elem[1][2] = (m_elem[1][0] * p.m_elem[0][2] +
			m_elem[1][1] * p.m_elem[1][2] +
			m_elem[1][2] * p.m_elem[2][2]);
		result.m_elem[2][0] = (m_elem[2][0] * p.m_elem[0][0] +
			m_elem[2][1] * p.m_elem[1][0] +
			m_elem[2][2] * p.m_elem[2][0]);
		result.m_elem[2][1] = (m_elem[2][0] * p.m_elem[0][1] +
			m_elem[2][1] * p.m_elem[1][1] +
			m_elem[2][2] * p.m_elem[2][1]);
		result.m_elem[2][2] = (m_elem[2][0] * p.m_elem[0][2] +
			m_elem[2][1] * p.m_elem[1][2] +
			m_elem[2][2] * p.m_elem[2][2]);
		return result;
	}

	// Computes ((*this)^T * p)
	// transform vector (x',y',0) = A * (x,y,0)
	void zero() {
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				m_elem[i][j] = (Scalar)0;
	}


	// build identity matrix
	void identity() {
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
				m_elem[i][j] = (Scalar)0;
			m_elem[i][i] = (Scalar)1;
		}
	}

	// returns true if all matrix m_elements are exactly equal to zero
	// determinant of 3x3 Matrix
	Scalar det() const
	{
		return ((m_elem[0][1] * m_elem[1][2] - m_elem[0][2] * m_elem[1][1]) * m_elem[2][0]
			+ (m_elem[0][2] * m_elem[1][0] - m_elem[0][0] * m_elem[1][2]) * m_elem[2][1]
			+ (m_elem[0][0] * m_elem[1][1] - m_elem[0][1] * m_elem[1][0]) * m_elem[2][2]);
	}

	// trace of a 3x3 Matrix
	Scalar trace() const {
		return m_elem[0][0] + m_elem[1][1] + m_elem[2][2];
	}

	// Transposed Matrix
	Matrix3x3T transposed() const {
		return(Matrix3x3T(m_elem[0][0], m_elem[1][0], m_elem[2][0],
			m_elem[0][1], m_elem[1][1], m_elem[2][1],
			m_elem[0][2], m_elem[1][2], m_elem[2][2]));
	}

	void transpose() {
		Scalar a = m_elem[1][0];
		m_elem[1][0] = m_elem[0][1];
		m_elem[0][1] = a;

		a = m_elem[2][0];
		m_elem[2][0] = m_elem[0][2];
		m_elem[0][2] = a;

		a = m_elem[2][1];
		m_elem[2][1] = m_elem[1][2];
		m_elem[1][2] = a;
	}

	Matrix3x3T<Scalar> inverse() const {
		Matrix3x3T<Scalar> M;

		computeInverseUnsave(M);

		return M;
	}

	// Returns the inverse of the matrix, or sets \c ok to \c false if the matrix is (nearly) singular
	Matrix3x3T<Scalar> inverse(bool &ok) const {
		Matrix3x3T<Scalar> M;

		ok = computeInverse(M, std::numeric_limits<Scalar>::epsilon());

		return M;
	}

	// Inverts the matrix or returns \c false if the matrix is (nearly) singular
	bool invert() {
		bool ok = false;
		*this = inverse(ok);
		return ok;
	}

	const Scalar *data() const { return &m_elem[0][0]; }

	// Cast to (const Scalar *). See \c data().
	operator const Scalar *() const { return data(); }

private:
	bool computeInverse(Matrix3x3T<Scalar> &M, Scalar eps) const {
		assert(&M != this);

		Scalar d = -m_elem[0][0] * m_elem[1][1] * m_elem[2][2]
			+ m_elem[0][0] * m_elem[1][2] * m_elem[2][1]
			+ m_elem[1][0] * m_elem[0][1] * m_elem[2][2]
			- m_elem[1][0] * m_elem[0][2] * m_elem[2][1]
			- m_elem[2][0] * m_elem[0][1] * m_elem[1][2]
			+ m_elem[2][0] * m_elem[0][2] * m_elem[1][1];

		if (fabs(d) <= eps)
		{
			// Need to make sure something gets written to \c M:
			M.setValues(std::numeric_limits<Scalar>::quiet_NaN());

			return false;
		}

		// d = 1/d;
		M(0, 0) = (m_elem[1][2] * m_elem[2][1] - m_elem[1][1] * m_elem[2][2]) / d;
		M(0, 1) = (m_elem[0][1] * m_elem[2][2] - m_elem[0][2] * m_elem[2][1]) / d;
		M(0, 2) = (m_elem[0][2] * m_elem[1][1] - m_elem[0][1] * m_elem[1][2]) / d;
		M(1, 0) = (m_elem[1][0] * m_elem[2][2] - m_elem[1][2] * m_elem[2][0]) / d;
		M(1, 1) = (m_elem[0][2] * m_elem[2][0] - m_elem[0][0] * m_elem[2][2]) / d;
		M(1, 2) = (m_elem[0][0] * m_elem[1][2] - m_elem[0][2] * m_elem[1][0]) / d;
		M(2, 0) = (m_elem[1][1] * m_elem[2][0] - m_elem[1][0] * m_elem[2][1]) / d;
		M(2, 1) = (m_elem[0][0] * m_elem[2][1] - m_elem[0][1] * m_elem[2][0]) / d;
		M(2, 2) = (m_elem[0][1] * m_elem[1][0] - m_elem[0][0] * m_elem[1][1]) / d;

		return true;
	}

	void computeInverseUnsave(Matrix3x3T<Scalar> &M) const {
		assert(&M != this);

		Scalar d = -m_elem[0][0] * m_elem[1][1] * m_elem[2][2]
			+ m_elem[0][0] * m_elem[1][2] * m_elem[2][1]
			+ m_elem[1][0] * m_elem[0][1] * m_elem[2][2]
			- m_elem[1][0] * m_elem[0][2] * m_elem[2][1]
			- m_elem[2][0] * m_elem[0][1] * m_elem[1][2]
			+ m_elem[2][0] * m_elem[0][2] * m_elem[1][1];

		// d = 1/d;
		M(0, 0) = (m_elem[1][2] * m_elem[2][1] - m_elem[1][1] * m_elem[2][2]) / d;
		M(0, 1) = (m_elem[0][1] * m_elem[2][2] - m_elem[0][2] * m_elem[2][1]) / d;
		M(0, 2) = (m_elem[0][2] * m_elem[1][1] - m_elem[0][1] * m_elem[1][2]) / d;
		M(1, 0) = (m_elem[1][0] * m_elem[2][2] - m_elem[1][2] * m_elem[2][0]) / d;
		M(1, 1) = (m_elem[0][2] * m_elem[2][0] - m_elem[0][0] * m_elem[2][2]) / d;
		M(1, 2) = (m_elem[0][0] * m_elem[1][2] - m_elem[0][2] * m_elem[1][0]) / d;
		M(2, 0) = (m_elem[1][1] * m_elem[2][0] - m_elem[1][0] * m_elem[2][1]) / d;
		M(2, 1) = (m_elem[0][0] * m_elem[2][1] - m_elem[0][1] * m_elem[2][0]) / d;
		M(2, 2) = (m_elem[0][1] * m_elem[1][0] - m_elem[0][0] * m_elem[1][1]) / d;
	}

private:
	// matrix elements
	Scalar m_elem[3][3];
};
