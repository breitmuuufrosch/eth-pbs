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
#define _USE_MATH_DEFINES
#include <math.h>
#include "Vector2T.h"

// Simple 2x2 matrix class
template <typename Scalar>
class Matrix2x2T
{
public:
	// Empty constructor - object remains uninitialized.
	Matrix2x2T() {}

	// Constructor using matrix elements a00 ... a11
	Matrix2x2T(Scalar a00, Scalar a01, Scalar a10, Scalar a11) {
		m_elem[0][0] = a00; m_elem[0][1] = a01;
		m_elem[1][0] = a10; m_elem[1][1] = a11;
	}

	// Copy-constructor
	Matrix2x2T(const Matrix2x2T &p) {
		m_elem[0][0] = p.m_elem[0][0];
		m_elem[0][1] = p.m_elem[0][1];
		m_elem[1][0] = p.m_elem[1][0];
		m_elem[1][1] = p.m_elem[1][1];
	}

	// Returns 2
	static int rows() { return 2; }

	// Returns 2
	static int cols() { return 2; }

	Scalar &operator() (int i, int j) {
		return m_elem[i][j];
	}

	const Scalar &operator() (int i, int j) const {
		return m_elem[i][j];
	}

	// +=-Operator
	Matrix2x2T &operator+=(const Matrix2x2T &p) {
		m_elem[0][0] += p.m_elem[0][0];
		m_elem[0][1] += p.m_elem[0][1];
		m_elem[1][0] += p.m_elem[1][0];
		m_elem[1][1] += p.m_elem[1][1];
		return *this;
	}

	// -=-Operator
	Matrix2x2T &operator-=(const Matrix2x2T &p) {
		m_elem[0][0] -= p.m_elem[0][0];
		m_elem[0][1] -= p.m_elem[0][1];
		m_elem[1][0] -= p.m_elem[1][0];
		m_elem[1][1] -= p.m_elem[1][1];
		return *this;
	}

	// /=-Operator
	Matrix2x2T &operator/=(Scalar s) {
		m_elem[0][0] /= s;
		m_elem[0][1] /= s;
		m_elem[1][0] /= s;
		m_elem[1][1] /= s;
		return *this;
	}

	// *=-Operator : Matrix * Scalar
	Matrix2x2T &operator*=(Scalar s) {
		m_elem[0][0] *= s;
		m_elem[0][1] *= s;
		m_elem[1][0] *= s;
		m_elem[1][1] *= s;
		return *this;
	}

	// *=-Operator : Matrix * Matrix
	Matrix2x2T &operator*=(const Matrix2x2T &p) {
		*this = *this * p;
		return *this;
	}

	// +-Operator
	Matrix2x2T operator+(const Matrix2x2T &p) const {
		return Matrix2x2T(
			m_elem[0][0] + p.m_elem[0][0],
			m_elem[0][1] + p.m_elem[0][1],
			m_elem[1][0] + p.m_elem[1][0],
			m_elem[1][1] + p.m_elem[1][1]);
	}

	// --Operator
	Matrix2x2T operator-(const Matrix2x2T &p) const {
		return Matrix2x2T(
			m_elem[0][0] - p.m_elem[0][0],
			m_elem[0][1] - p.m_elem[0][1],
			m_elem[1][0] - p.m_elem[1][0],
			m_elem[1][1] - p.m_elem[1][1]);
	}

	//-Operator
	Matrix2x2T operator/(Scalar s) const {
		return Matrix2x2T(
			m_elem[0][0] / s, m_elem[0][1] / s,
			m_elem[1][0] / s, m_elem[1][1] / s);
	}

	// *-Operator : Matrix * Scalar
	Matrix2x2T operator*(Scalar s) const {
		return Matrix2x2T(
			m_elem[0][0] * s, m_elem[0][1] * s,
			m_elem[1][0] * s, m_elem[1][1] * s);
	}

	// friend *-Operator : Scalar * Matrix
	friend Matrix2x2T operator*(Scalar s, const Matrix2x2T &p) {
		return Matrix2x2T(
			p.m_elem[0][0] * s, p.m_elem[0][1] * s,
			p.m_elem[1][0] * s, p.m_elem[1][1] * s);
	}

	// *-Operator : Matrix * Vector
	Vector2T<Scalar> operator*(const Vector2T<Scalar> &vec) const {
		return Vector2T<Scalar>(
			m_elem[0][0] * vec[0] + m_elem[0][1] * vec[1],
			m_elem[1][0] * vec[0] + m_elem[1][1] * vec[1]);
	}

	// *-Operator : Matrix * Matrix
	Matrix2x2T operator*(const Matrix2x2T &p) const {
		Matrix2x2T result;
		result.m_elem[0][0] = m_elem[0][0] * p.m_elem[0][0] + m_elem[0][1] * p.m_elem[1][0];
		result.m_elem[0][1] = m_elem[0][0] * p.m_elem[0][1] + m_elem[0][1] * p.m_elem[1][1];
		result.m_elem[1][0] = m_elem[1][0] * p.m_elem[0][0] + m_elem[1][1] * p.m_elem[1][0];
		result.m_elem[1][1] = m_elem[1][0] * p.m_elem[0][1] + m_elem[1][1] * p.m_elem[1][1];

		return result;
	}

	// Sets all elements of the matrix to zero
	void setZero() {
		for (int i = 0; i < 2; i++)
			for (int j = 0; j < 2; j++)
				m_elem[i][j] = 0;
	}

	// Sets the matrix to the 2x2 identity matrix
	void setIdentity() {
		for (int i = 0; i < 2; i++)
		{
			for (int j = 0; j < 2; j++)
			{
				m_elem[i][j] = 0;
			}

			m_elem[i][i] = 1;
		}
	}

	// Returns the determinant of the 2x2 matrix
	Scalar det() const {
		return m_elem[0][0] * m_elem[1][1] - m_elem[0][1] * m_elem[1][0];
	}

	// Returns the trace of the 2x2 matrix
	Scalar trace() const {
		return m_elem[0][0] + m_elem[1][1];
	}

	// Returns the transposed matrix
	Matrix2x2T transposed() const {
		return Matrix2x2T(
			m_elem[0][0], m_elem[1][0],
			m_elem[0][1], m_elem[1][1]);
	}

	// Returns the inverse matrix
	Matrix2x2T inverse() const {
		Scalar a = (*this)(0, 0);
		Scalar b = (*this)(0, 1);
		Scalar c = (*this)(1, 0);
		Scalar d = (*this)(1, 1);

		Scalar det = a*d - b*c;

		Matrix2x2T M;
		M(0, 0) = d;
		M(0, 1) = -b;
		M(1, 0) = -c;
		M(1, 1) = a;

		return M * (1.0f / det);
	}

private:
	// Matrix elements
	Scalar m_elem[2][2];
};

// helper for printing a vector
template<class T>
std::ostream& operator<<(std::ostream& os, const  Matrix2x2T<T> &mat)
{
	char buf[256];
	//snprintf(buf,256,"{%f,%f,%f,%f}", (double)mat(0,0), (double)mat(1,0), (double)mat(0,1), (double)mat(1,1)   );
	sprintf(buf, "{%f,%f,%f,%f}", (double)mat(0, 0), (double)mat(1, 0), (double)mat(0, 1), (double)mat(1, 1));
	os << std::string(buf);
	return os;
}
