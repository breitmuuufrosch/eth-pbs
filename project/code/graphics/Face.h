#pragma once

#include <Eigen/Core>

namespace pbs17 {
	class Face {
	public:
		Face()
			: Face(Eigen::Vector3i(0, 0, 0), std::vector<Eigen::Vector3d>(), 0, Eigen::Vector3d(1, 0, 0)) {}

		Face(Eigen::Vector3i indices, std::vector<Eigen::Vector3d> vertices, double distance, Eigen::Vector3d normal)
			: _indices(indices), _vertices(vertices), _distance(distance), _normal(normal) {}

		Eigen::Vector3i getIndices() const {
			return _indices;
		}

		void setIndices(const Eigen::Vector3i indices) {
			_indices = indices;
		}

		Eigen::Vector3d getVertex(const int pos) const {
			return _vertices[pos];
		}

		void setVertices(const std::vector<Eigen::Vector3d> vertices) {
			_vertices = vertices;
		}

		double getDistance() const {
			return _distance;
		}

		void setDistance(const double distance) {
			_distance = distance;
		}

		Eigen::Vector3d getNormal() const {
			return _normal;
		}

		void setNormal(const Eigen::Vector3d normal) {
			_normal = normal;
		}
		
	private:
		Eigen::Vector3i _indices;
		std::vector<Eigen::Vector3d> _vertices;
		double _distance;
		Eigen::Vector3d _normal;
	};
}
