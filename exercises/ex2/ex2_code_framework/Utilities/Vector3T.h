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

// Simple 3D vector class
template<typename T>
class Vector3T
{
public:
	//----------------------------------------------------------- constructors
	Vector3T() { m_xyz[0] = (T)0; m_xyz[1] = (T)0; m_xyz[2] = (T)0; }
	explicit Vector3T(T v) { m_xyz[0] = v; m_xyz[1] = v; m_xyz[2] = v; }
	Vector3T(T x, T y, T z) { m_xyz[0] = x; m_xyz[1] = y; m_xyz[2] = z; }
	explicit Vector3T(const T* xyz) { m_xyz[0] = xyz[0]; m_xyz[1] = xyz[1]; m_xyz[2] = xyz[2]; }
	Vector3T(const Vector3T<T> &other) { *this = other; }

	//----------------------------------------------------------- cast internal data type
	template <typename Scalar2>
	explicit Vector3T(const Vector3T<Scalar2> &m) {
		for (int r = 0; r < m.dim(); r++)
		{
			(*this)[r] = static_cast<T> (m[r]);
		}
	}

	//----------------------------------------------------------- assignment
	Vector3T<T> & operator=(const Vector3T<T> &other) { m_xyz[0] = other.m_xyz[0]; m_xyz[1] = other.m_xyz[1]; m_xyz[2] = other.m_xyz[2]; return *this; }

	//----------------------------------------------------------- element access
	// read-write
	T& operator[](int i) { assert(i >= 0 && i < dim()); return m_xyz[i]; }
	T& x() { return m_xyz[0]; }
	T& y() { return m_xyz[1]; }
	T& z() { return m_xyz[2]; }

	// read-only
	const T& operator[](int i) const { assert(i >= 0 && i < dim()); return m_xyz[i]; }
	const T& x() const { return m_xyz[0]; }
	const T& y() const { return m_xyz[1]; }
	const T& z() const { return m_xyz[2]; }

	// write only
	void setX(T x) { m_xyz[0] = x; }
	void setY(T y) { m_xyz[1] = y; }
	void setZ(T z) { m_xyz[2] = z; }

	void xyz(T* xyz) const { xyz[0] = m_xyz[0]; xyz[1] = m_xyz[1]; xyz[2] = m_xyz[2]; }
	void setXYZ(const T* xyz) { m_xyz[0] = xyz[0]; m_xyz[1] = xyz[1]; m_xyz[2] = xyz[2]; }

	//----------------------------------------------------------- logic operations
	bool isNullVector() const { return (m_xyz[0] == (T)0) && (m_xyz[1] == (T)0) && (m_xyz[2] == (T)0); }

	bool operator==(const Vector3T<T> &other) const {
		if (x() != other.x()) return false;
		if (y() != other.y()) return false;
		if (z() != other.z()) return false;
		return true;
	}

	bool operator!=(const Vector3T<T> &other) const {
		return !(*this == other);
	}

	// Lexicographical comparison
	bool operator<(const Vector3T<T> &other) const {
		for (int i = 0; i < 3; i++)
		{
			if ((*this)[i] != other[i])
				return (*this)[i] < other[i];
		}

		return false;
	}

	// Lexicographical comparison
	bool operator>(const Vector3T<T> &other) const {
		for (int i = 0; i < 3; i++)
		{
			if ((*this)[i] != other[i])
				return (*this)[i] > other[i];
		}

		return false;
	}

	//----------------------------------------------------------- basic arithmetic operations
	//----------------------------------------------------------- scalar arithmetic operations
	Vector3T<T> operator*(T s) const {
		return Vector3T<T>(m_xyz[0] * s, m_xyz[1] * s, m_xyz[2] * s);
	}

	Vector3T<T>& operator*=(T s) {
		m_xyz[0] *= s;
		m_xyz[1] *= s;
		m_xyz[2] *= s;
		return *this;
	}

	Vector3T<T> operator/(T s) const {
		return Vector3T<T>(m_xyz[0] / s, m_xyz[1] / s, m_xyz[2] / s);
	}

	Vector3T<T>& operator/=(T s) {
		m_xyz[0] /= s;
		m_xyz[1] /= s;
		m_xyz[2] /= s;
		return *this;
	}

	//----------------------------------------------------------- vector arithmetic operations
	Vector3T<T> operator+(const Vector3T<T> &v2) const {
		return Vector3T<T>(x() + v2.x(), y() + v2.y(), z() + v2.z());
	}

	Vector3T<T>& operator+=(const Vector3T<T> &v) {
		m_xyz[0] += v.x();
		m_xyz[1] += v.y();
		m_xyz[2] += v.z();
		return *this;
	}

	Vector3T<T> operator-(const Vector3T<T> &v2) const {
		return Vector3T<T>(x() - v2.x(), y() - v2.y(), z() - v2.z());
	}

	Vector3T<T>& operator-=(const Vector3T<T> &v) {
		m_xyz[0] -= v.x();
		m_xyz[1] -= v.y();
		m_xyz[2] -= v.z();
		return *this;
	}

	Vector3T<T> operator-() const {
		return Vector3T<T>(-m_xyz[0], -m_xyz[1], -m_xyz[2]);
	}

	Vector3T<T> componentWiseProduct(const Vector3T<T> &v2) const {
		return Vector3T<T>(x()*v2.x(), y()*v2.y(), z()*v2.z());
	}

	Vector3T<T> componentWiseDivision(const Vector3T<T> &v2) const {
		return Vector3T<T>(x() / v2.x(), y() / v2.y(), z() / v2.z());
	}

	// Inner product (dot product)
	T operator*(const Vector3T<T> &v2) const {
		return x()*v2.x() + y()*v2.y() + z()*v2.z();
	}

	// Inner product (dot product)
	T operator|(const Vector3T<T> &v2) const {
		return (*this) * v2;
	}

	// Cross product
	Vector3T<T> cross(const Vector3T<T> &v2) const {
		return Vector3T<T>(y()*v2.z() - v2.y()*z(), z()*v2.x() - v2.z()*x(), x()*v2.y() - v2.x()*y());
	}

	// Cross product
	Vector3T<T> operator%(const Vector3T<T> &v2) const {
		return cross(v2);
	}

	//----------------------------------------------------------- other
	void normalize() {
		*this /= length();
	}

	Vector3T<T> normalized() const {
		return *this / length();
	}

	T length() const {
		return sqrt(squaredLength());
	}

	T norm() const {
		return length();
	}

	T squaredLength() const
	{
		return m_xyz[0] * m_xyz[0] + m_xyz[1] * m_xyz[1] + m_xyz[2] * m_xyz[2];
	}

	T sqrnorm() const {
		return squaredLength();
	}

	// Returns an arbitrary vector which is orthogonal to this vector
	Vector3T<T> orthogonalVector() const {
		Vector3T<T> vecNorm = normalized();

		const int dim = 3;

		int minIdx = -1;
		T minDot = 0;

		for (int i = 0; i < dim; i++)
		{
			// Create i-th basis vector:
			Vector3T<T> basisVec;
			for (int j = 0; j < dim; j++)
				basisVec[j] = (j == i);

			T dotProd = vecNorm | basisVec;

			if (minIdx == -1 || fabs(dotProd) < fabs(minDot))
			{
				minIdx = i;
				minDot = dotProd;
			}
		}

		// Create minIdx-th basis vector:
		Vector3T<T> basisVec;
		for (int j = 0; j < dim; j++)
			basisVec[j] = (j == minIdx);

		return basisVec - vecNorm * minDot;
	}

	// Returns the minimal component
	T minComponent() const {
		T m = (*this)[0];

		for (int i = 1; i < 3; i++)
		{
			if ((*this)[i] < m)
				m = (*this)[i];
		}

		return m;
	}

	// Returns the maximal component
	T maxComponent() const {
		T m = (*this)[0];

		for (int i = 1; i < 3; i++)
		{
			if ((*this)[i] > m)
				m = (*this)[i];
		}

		return m;
	}

	// Component-wise minimum
	Vector3T<T> componentWiseMin(const Vector3T<T> &v2) const {
		Vector3T<T> v(*this);
		v.minimize(v2);
		return v;
	}

	// Component-wise maximum
	Vector3T<T> componentWiseMax(const Vector3T<T> &v2) const {
		Vector3T<T> v(*this);
		v.maximize(v2);
		return v;
	}

	// Minimizes values: same as *this = min(*this, v2)
	void minimize(const Vector3T<T> &v2) {
		for (int i = 0; i < 3; i++)
		{
			if (v2[i] < (*this)[i])
				(*this)[i] = v2[i];
		}
	}

	// Maximizes values: same as *this = max(*this, v2)
	void maximize(const Vector3T<T> &v2) {
		for (int i = 0; i < 3; i++)
		{
			if (v2[i] > (*this)[i])
				(*this)[i] = v2[i];
		}
	}

	// Compute the reflection of this vector with \c n
	Vector3T<T> reflectionAt(const Vector3T<T> &n) const {
		return *this - 2 * (n*(*this))*n;
	}

	// Returns a pointer to the raw vector data of length 3
	const T *data() const { return m_xyz; }

	// Cast to (const T *). See \c data().
	operator const T *() const { return data(); }

	//== STATIC MEMBERS =========================================================
public:
	static int dim() { return 3; }

	static const Vector3T<T> &zero() {
		static const Vector3T<T> zeroVec(0, 0, 0);
		return zeroVec;
	}

private:
	T m_xyz[3];
};

//== NON MEMBER DEFINITION =========================================================
// scalar * vector
template<class T>
Vector3T<T> operator*(T const &s, Vector3T<T> const &v)
{
	return v*s;
}
