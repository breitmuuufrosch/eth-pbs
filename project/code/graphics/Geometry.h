#pragma once

#include <Eigen/Core>

namespace pbs17 {
	inline bool isSameDirection(Eigen::Vector3d v1, Eigen::Vector3d v2) {
		return v1.dot(v2) > 0;
	}

	inline bool isOppositeDirection(Eigen::Vector3d v1, Eigen::Vector3d v2) {
		return v1.dot(v2) < 0;
	}

	inline Eigen::Vector3d getNormalFromPoints(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c) {
		return (a - b).cross(c - b);
	}

	inline Eigen::Vector3d barycentric(const Eigen::Vector3d &p, const Eigen::Vector3d &a, const Eigen::Vector3d &b, const Eigen::Vector3d &c) {
		// code from Crister Erickson's Real-Time Collision Detection
		Eigen::Vector3d v0 = b - a, v1 = c - a, v2 = p - a;
		double d00 = v0.dot(v0);
		double d01 = v0.dot(v1);
		double d11 = v1.dot(v1);
		double d20 = v2.dot(v0);
		double d21 = v2.dot(v1);
		double denom = d00 * d11 - d01 * d01;

		double v = (d11 * d20 - d01 * d21) / denom;
		double w = (d00 * d21 - d01 * d20) / denom;
		double u = 1.0f - v - w;

		return Eigen::Vector3d(u, v, w);
	}
}