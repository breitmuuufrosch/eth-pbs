#include <osg/MatrixTransform>
#include <Eigen/Core>

namespace pbs17 {
	/// Convert 2D vector of floats to OSG
	inline osg::Vec2f toOsg(const Eigen::Vector2f& vector) {
		osg::Vec2f osgVector;
		Eigen::Map<Eigen::Vector2f>(osgVector.ptr()) = vector;
		return osgVector;
	}
	/// Convert from OSG to 2D vector of floats
	inline Eigen::Vector2f fromOsg(const osg::Vec2f& vector) {
		return Eigen::Vector2f(vector.ptr());
	}

	/// Convert 2D vector of doubles to OSG
	inline osg::Vec2d toOsg(const Eigen::Vector2d& vector) {
		osg::Vec2d osgVector;
		Eigen::Map<Eigen::Vector2d>(osgVector.ptr()) = vector;
		return osgVector;
	}
	/// Convert from OSG to 2D vector of doubles
	inline Eigen::Vector2d fromOsg(const osg::Vec2d& vector) {
		return Eigen::Vector2d(vector.ptr());
	}

	/// Convert 3D vector of floats to OSG
	inline osg::Vec3f toOsg(const Eigen::Vector3f& vector) {
		osg::Vec3f osgVector;
		Eigen::Map<Eigen::Vector3f>(osgVector.ptr()) = vector;
		return osgVector;
	}
	/// Convert from OSG to 3D vector of floats
	inline Eigen::Vector3f fromOsg(const osg::Vec3f& vector) {
		return Eigen::Vector3f(vector.ptr());
	}

	/// Convert 3D vector of doubles to OSG
	inline osg::Vec3d toOsg(Eigen::Vector3d vector) {
		osg::Vec3d osgVector;
		Eigen::Map<Eigen::Vector3d>(osgVector.ptr()) = vector;
		return osgVector;
	}
	/// Convert from OSG to 3D vector of doubles
	inline Eigen::Vector3d fromOsg(const osg::Vec3d& vector) {
		return Eigen::Vector3d(vector.ptr());
	}


	/// Convert 4D vector of floats to OSG
	inline osg::Vec4f toOsg(const Eigen::Vector4f& vector) {
		osg::Vec4f osgVector;
		Eigen::Map<Eigen::Vector4f>(osgVector.ptr()) = vector;
		return osgVector;
	}
	/// Convert from OSG to 4D vector of floats
	inline Eigen::Vector4f fromOsg(const osg::Vec4f& vector) {
		return Eigen::Vector4f(vector.ptr());
	}

	/// Convert 4D vector of doubles to OSG
	inline osg::Vec4d toOsg(const Eigen::Vector4d& vector) {
		osg::Vec4d osgVector;
		Eigen::Map<Eigen::Vector4d>(osgVector.ptr()) = vector;
		return osgVector;
	}
	/// Convert from OSG to 4D vector of doubles
	inline Eigen::Vector4d fromOsg(const osg::Vec4d& vector) {
		return Eigen::Vector4d(vector.ptr());
	}

}
