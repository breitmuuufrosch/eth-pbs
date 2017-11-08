//=============================================================================
//  Physically-based Simulation in Computer Graphics
//  ETH Zurich
//
//  Author: Peter Kaufmann, Christian Schumacher
//=============================================================================

#pragma once

#include <iostream>
#include <cstdio>
#define _USE_MATH_DEFINES
#include <math.h>
#include <assert.h>

// Simple 2D vector class
template<typename T>
class Vector2T
{
public:
	//----------------------------------------------------------- constructors
	Vector2T()
	{
		m_xy[0] = (T)0;
		m_xy[1] = (T)0;
	}

	Vector2T(T v)
	{
		m_xy[0] = v;
		m_xy[1] = v;
	}

	Vector2T(const T x, const T y)
	{
		m_xy[0] = x;
		m_xy[1] = y;
	}

	Vector2T(const Vector2T<T> &other)
	{
		*this = other;
	}

	//----------------------------------------------------------- assignment
	Vector2T<T> & operator=(const Vector2T<T> &other)
	{
		m_xy[0] = other.m_xy[0]; m_xy[1] = other.m_xy[1]; return *this;
	}

	//----------------------------------------------------------- element access
	//  read-write
	T& operator[](int i) { assert(i >= 0 && i < dim()); return m_xy[i]; }
	// NT
	T& x() { return m_xy[0]; }
	const T& x() const { return m_xy[0]; }
	T& y() { return m_xy[1]; }
	const T& y() const { return m_xy[1]; }

	//  read-only
	const T& operator[](int i) const { assert(i >= 0 && i < dim()); return m_xy[i]; }

	bool operator==(const Vector2T<T> &other) const
	{
		if (x() != other.x()) return false;
		if (y() != other.y()) return false;
		return true;
	}

	bool operator!=(const Vector2T<T> &other) const
	{
		return !(*this == other);
	}

	//----------------------------------------------------------- basic arithmetic operations
	//----------------------------------------------------------- scalar arithmetic operations
	Vector2T<T> operator*(T s) const
	{
		return Vector2T<T>(m_xy[0] * s, m_xy[1] * s);
	}

	Vector2T<T>& operator*=(T s)
	{
		m_xy[0] = m_xy[0] * s;
		m_xy[1] = m_xy[1] * s;
		return *this;
	}

	Vector2T<T> operator/(T s) const
	{
		return Vector2T<T>(m_xy[0] / s, m_xy[1] / s);
	}

	Vector2T<T>& operator/=(T s)
	{
		m_xy[0] = m_xy[0] / s;
		m_xy[1] = m_xy[1] / s;
		return *this;
	}

	//----------------------------------------------------------- vector arithmetic operations
	Vector2T<T> operator+(const Vector2T<T> &v2) const
	{
		return Vector2T<T>(x() + v2.x(), y() + v2.y());
	}

	Vector2T<T>& operator+=(const Vector2T<T> &v)
	{
		m_xy[0] += v.x();
		m_xy[1] += v.y();
		return *this;
	}

	Vector2T<T> operator-(const Vector2T<T> &v2) const
	{
		return Vector2T<T>(x() - v2.x(), y() - v2.y());
	}

	Vector2T<T>& operator-=(const Vector2T<T> &v)
	{
		m_xy[0] -= v.x();
		m_xy[1] -= v.y();
		return *this;
	}

	Vector2T<T> operator-() const
	{
		return Vector2T<T>(-m_xy[0], -m_xy[1]);
	}

	// inner product (dot product)
	T operator|(const Vector2T<T> &v2) const
	{
		return (*this)[0] * v2[0] + (*this)[1] * v2[1];
	}

	//----------------------------------------------------------- other
	Vector2T<T> normalized() const
	{
		return *this / length();
	}

	T norm() const
	{
		return length();
	}

	T sqrnorm() const
	{
		return squaredLength();
	}

	T length() const
	{
		return sqrt(x()*x() + y()*y());
	}

	T squaredLength() const
	{
		return (x()*x() + y()*y());
	}

	static int dim() { return 2; }

private:
	T m_xy[2];
};

// scalar * vector
template<class T>
Vector2T<T> operator*(T const &s, Vector2T<T> const &v)
{
	return v*s;
}

// helper for printing a vector
template<class T>
std::ostream& operator<<(std::ostream& os, const  Vector2T<T> &v2)
{
	char buf[256];
	//snprintf(buf,256,"[%f,%f]", (double)v2[0],(double)v2[1]);
	sprintf(buf, "[%f,%f]", (double)v2[0], (double)v2[1]);
	os << std::string(buf);
	//os << '[' << i[0] << ", " << i[1] << ", " << i[2] << ']';
	return os;
}

typedef Vector2T<double> Vec2;
