#pragma once

#include <osg/MatrixTransform>
#include <Eigen/Core>

namespace pbs17 {

	/// Convert a fixed-size 2x2 matrix of floats to OSG.
	template <int MOpt> inline
		const osg::Matrix2 toOsg(const Eigen::Matrix<float, 2, 2, MOpt>& matrix) {
		osg::Matrix2 osgMatrix;
		Eigen::Map<Eigen::Matrix<float, 2, 2, Eigen::ColMajor> >(osgMatrix.ptr()) = matrix;
		return osgMatrix;
	}

	/// Convert a fixed-size 2x2 matrix of doubles to OSG.
	template <int MOpt> inline
		const osg::Matrix2d toOsg(const Eigen::Matrix<double, 2, 2, MOpt>& matrix) {
		osg::Matrix2d osgMatrix;
		Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::ColMajor> >(osgMatrix.ptr()) = matrix;
		return osgMatrix;
	}

	/// Convert from OSG to a 2x2 matrix of floats.
	inline const Eigen::Matrix<float, 2, 2, Eigen::RowMajor> fromOsg(const osg::Matrix2& matrix) {
		return Eigen::Map<const Eigen::Matrix<float, 2, 2, Eigen::ColMajor> >(matrix.ptr());
	}
	/// Convert from OSG to a 2x2 matrix of doubles.
	inline const Eigen::Matrix<double, 2, 2, Eigen::RowMajor> fromOsg(const osg::Matrix2d& matrix) {
		return Eigen::Map<const Eigen::Matrix<double, 2, 2, Eigen::ColMajor> >(matrix.ptr());
	}

	/// Convert a fixed-size 3x3 matrix of floats to OSG.
	template <int MOpt> inline
		const osg::Matrix3 toOsg(const Eigen::Matrix<float, 3, 3, MOpt>& matrix) {
		osg::Matrix3 osgMatrix;
		Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::ColMajor> >(osgMatrix.ptr()) = matrix;
		return osgMatrix;
	}

	/// Convert a fixed-size 3x3 matrix of doubles to OSG.
	template <int MOpt> inline
		const osg::Matrix3d toOsg(const Eigen::Matrix<double, 3, 3, MOpt>& matrix) {
		osg::Matrix3d osgMatrix;
		Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::ColMajor> >(osgMatrix.ptr()) = matrix;
		return osgMatrix;
	}

	/// Convert from OSG to a 3x3 matrix of floats.
	inline const Eigen::Matrix<float, 3, 3, Eigen::RowMajor> fromOsg(const osg::Matrix3& matrix) {
		return Eigen::Map<const Eigen::Matrix<float, 3, 3, Eigen::ColMajor> >(matrix.ptr());
	}

	/// Convert from OSG to a 3x3 matrix of doubles.
	inline const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> fromOsg(const osg::Matrix3d& matrix) {
		return Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::ColMajor> >(matrix.ptr());
	}

	/// Convert a fixed-size 4x4 matrix of floats to OSG.
	template <int MOpt> inline
		const osg::Matrixf toOsg(const Eigen::Matrix<float, 4, 4, MOpt>& matrix) {
		osg::Matrixf osgMatrix;
		Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::ColMajor> >(osgMatrix.ptr()) = matrix;
		return osgMatrix;
	}

	/// Convert from OSG to a 4x4 matrix of floats.
	inline const Eigen::Matrix<float, 4, 4, Eigen::RowMajor> fromOsg(const osg::Matrixf& matrix) {
		return Eigen::Map<const Eigen::Matrix<float, 4, 4, Eigen::ColMajor> >(matrix.ptr());
	}

	/// Convert a fixed-size 4x4 matrix of doubles to OSG.
	template <int MOpt> inline
		const osg::Matrixd toOsg(const Eigen::Matrix<double, 4, 4, MOpt>& matrix) {
		osg::Matrixd osgMatrix;
		Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor> >(osgMatrix.ptr()) = matrix;
		return osgMatrix;
	}

	/// Convert from OSG to a 4x4 matrix of doubles.
	inline const Eigen::Matrix<double, 4, 4, Eigen::RowMajor> fromOsg(const osg::Matrixd& matrix) {
		return Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::ColMajor> >(matrix.ptr());
	}

}
