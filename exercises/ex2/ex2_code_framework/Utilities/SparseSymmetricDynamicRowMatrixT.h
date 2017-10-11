//=============================================================================
//  Physically-based Simulation in Computer Graphics
//  ETH Zurich
//
//  Author: Peter Kaufmann, Christian Schumacher
//=============================================================================

#pragma once

#include <iostream>
#include <string>
#include <map>
#include <cstdio>
#include <assert.h>

/*! A symmetric sparse matrix, using dynamic data structures to allow fill-ins. Slow but flexible.
	NOTE: Only the lower-triagonal elements of the matrix are stored and only
	those entries can be accessed! */
template<class T>
class SparseSymmetricDynamicRowMatrixT
{
public:
	/*! Constructor. The matrix will have \c numRowsCols rows and \c numRowsCols columns.
		Only the lower-triagonal elements of the symmetric matrix will be stored. */
	SparseSymmetricDynamicRowMatrixT(int numRowsCols) {
		m_numCols = numRowsCols;
		m_rowData.resize(numRowsCols);
	}

	/*! Constructor. The matrix will have 0 rows and 0 columns.
		Only the lower-triagonal elements of the matrix will be stored. */
	SparseSymmetricDynamicRowMatrixT() {
		m_numCols = 0;
	}

	// Removes all elements and resizes the matrix to size (0, 0)
	void Clear() {
		m_numCols = 0;
		m_rowData.clear();
	}

	virtual ~SparseSymmetricDynamicRowMatrixT() {}

	// Resizes the matrix to \c numRowsCols rows and \c numRowsCols columns and sets all elements to zero
	void ClearResize(int numRowsCols) {
		Clear();
		m_numCols = numRowsCols;
		m_rowData.resize(numRowsCols);
	}

	// Computes: b = this * x
	void MultVector(const std::vector<T> &x, std::vector<T> &b) const {
		// Set b to zero:
		for (int i = 0; i < (int)b.size(); i++)
			b[i] = 0;

		int nrows = GetNumRows();

		for (int row = 0; row < nrows; row++)
		{
			const std::map<int, T> &rowData = m_rowData[row];

			T rowSum = 0;

			for (typename std::map<int, T>::const_iterator iter = rowData.begin(); iter != rowData.end(); iter++)
			{
				int col = iter->first;
				assert(col <= row);

				T val = iter->second;

				rowSum += val * x[col];

				if (col < row)
					b[col] += val * x[row];
			}

			b[row] += rowSum;
		}
	}

	/*! Modifies the matrix and the vector \c b so the linear system 'this * x = b' will have the solution \c value at index \c idx. The row
		\c idx of this matrix must be occupied. */
	void FixSolution(std::vector<T> &b, int idx, T value) {
		int n = (int)b.size();
		assert(GetNumCols() == n);
		assert(GetNumRows() == n);
		assert(idx >= 0 && idx < n);

		std::map<int, T> &rowData = m_rowData[idx];

		// Need at least one entry in row 'idx':
		assert(rowData.size() > 0);

		for (typename std::map<int, T>::iterator iter = rowData.begin(); iter != rowData.end(); iter++)
		{
			int col = iter->first;
			assert(col <= idx);

			b[col] -= iter->second * value;

			if (col == idx)
				iter->second = 1;
			else
				iter->second = 0;
		}

		b[idx] = value;

		for (int i = idx + 1; i < n; i++)
		{
			T oldValue = GetAt(i, idx);
			if (oldValue != 0)
			{
				b[i] -= oldValue * value;
				GetAt(i, idx) = 0;
			}
		}
	}

	/*! Returns the element at (\c row, \c col). Only elements in the lower-triagonal part
		of the matrix can be accessed! */
	const T &operator()(int row, int col) const {
		return GetAt(row, col);
	}

	/*! Returns the element at (\c row, \c col). Only elements in the lower-triagonal part
		of the matrix can be accessed! */
	T &operator()(int row, int col) {
		return GetAt(row, col);
	}

	/*! Returns the element at (\c row, \c col). Only elements in the lower-triagonal part
		of the matrix can be accessed! */
	const T &GetAt(int row, int col) const {
		assert(row >= 0 && row < GetNumRows());
		assert(col >= 0 && col < GetNumCols());

		assert(row >= col);

		const std::map<int, T> &rowData = m_rowData[row];

		typename std::map<int, T>::const_iterator iter = rowData.find(col);
		if (iter == rowData.end())
			return m_zero;

		return iter->second;
	}

	/*! Returns the element at (\c row, \c col). Only elements in the lower-triagonal part
		of the matrix can be accessed! */
	T &GetAt(int row, int col) {
		assert(row >= 0 && row < GetNumRows());
		assert(col >= 0 && col < GetNumCols());

		assert(row >= col);

		std::map<int, T> &rowData = m_rowData[row];

		typename std::map<int, T>::iterator iter = rowData.find(col);
		if (iter == rowData.end())
		{
			// Q_ASSERT(FALSE);	// Element does not exist!
			// return *(T*)NULL;

			// fill-in:
			rowData[col] = 0;
			iter = rowData.find(col);
			assert(iter != rowData.end());
		}

		return iter->second;
	}

	// Returns the element at (\c row, \c col). Elements from the upper-triagonal part are returned as well.
	const T &GetAtFull(int row, int col) const {
		if (row < col)
		{
			// Swap row and col:
			int tmp = row;
			row = col;
			col = tmp;
		}

		return GetAt(row, col);
	}

	// Returns the number of rows of the matrix
	int GetNumRows() const { return m_rowData.size(); }

	// Returns the number of columns of the matrix
	int GetNumCols() const { return m_numCols; }

	// Returns the number of non-zero entries. The values above the diagonal are not counted.
	int GetNumNonZero() const {
		int nnz = 0;
		for (int i = 0; i < m_rowData.size(); i++)
		{
			nnz += m_rowData[i].size();
		}

		return nnz;
	}

private:
	int m_numCols;
	std::vector<std::map<int, T> > m_rowData;

	static T m_zero;
};

template<class T>
T SparseSymmetricDynamicRowMatrixT<T>::m_zero = 0;
