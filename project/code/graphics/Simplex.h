#pragma once

#include <algorithm>

#include <Eigen/Core>
#include <osg/Geometry>

#include "CGAL.h"

namespace pbs17 {

	/**
	 * \brief Class which represents the convex-hull and manages the structures between the different frameworks.
	 */
	class Simplex {
	public:
		int count() {
			return _vertices.size();
		}

		Eigen::Vector3d operator[](int i) {
			return _vertices[i];
		}

		Simplex(std::vector<Eigen::Vector3d> vertices)
			: _vertices(vertices) {}

		void add(Eigen::Vector3d v) {
			_vertices.push_back(v);
		}

		void remove(Eigen::Vector3d v) {
			_vertices.erase(std::remove(_vertices.begin(), _vertices.end(), v));
		}

	private:
		std::vector<Eigen::Vector3d> _vertices;
	};
}
