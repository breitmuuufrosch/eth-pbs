#pragma once

#include <algorithm>

#include <Eigen/Core>

#include "Face.h"
#include "Geometry.h"
#include "Edge.h"

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

		void triangulate() {
			switch (_vertices.size()) {
			case 2:
			{
				Eigen::Vector3d o = Eigen::Vector3d(0, 0, 0);
				Eigen::Vector3d a = _vertices[0];
				Eigen::Vector3d b = _vertices[1];
				Eigen::Vector3d toC = (a - b).cross(o - b);
				_vertices.push_back(a + toC);

				if (isCorrectOrder(0, 1, 2, o))
				{
					_triangles.push_back(Eigen::Vector3i(0, 1, 2));
				} else {
					_triangles.push_back(Eigen::Vector3i(0, 2, 1));
				}

				break;
			}
			case 3:
			{
				Eigen::Vector3d o = Eigen::Vector3d(0, 0, 0);

				if (isCorrectOrder(0, 1, 2, o)) {
					_triangles.push_back(Eigen::Vector3i(0, 1, 2));
				} else {
					_triangles.push_back(Eigen::Vector3i(0, 2, 1));
				}

				break;
			}
			case 4:
			{
				if (isCorrectOrder(0, 1, 2, 3)) {
					_triangles.push_back(Eigen::Vector3i(0, 1, 2));
				} else {
					_triangles.push_back(Eigen::Vector3i(0, 2, 1));
				}

				if (isCorrectOrder(0, 1, 3, 2)) {
					_triangles.emplace_back(Eigen::Vector3i(0, 1, 3));
				} else {
					_triangles.push_back(Eigen::Vector3i(0, 3, 1));
				}

				if (isCorrectOrder(0, 2, 3, 1)) {
					_triangles.push_back(Eigen::Vector3i(0, 2, 3));
				} else {
					_triangles.push_back(Eigen::Vector3i(0, 3, 2));
				}

				if (isCorrectOrder(1, 2, 3, 0)) {
					_triangles.push_back(Eigen::Vector3i(1, 2, 3));
				} else {
					_triangles.push_back(Eigen::Vector3i(1, 3, 2));
				}
				break;
			}
			}
		}

		Face findClosestFace() {
			Face closest;
			closest.setDistance(DBL_MAX);

			for (int i = 0; i < _triangles.size(); ++i) {
				Eigen::Vector3d a = _vertices[_triangles[i].x()];
				Eigen::Vector3d b = _vertices[_triangles[i].y()];
				Eigen::Vector3d c = _vertices[_triangles[i].z()];
				Eigen::Vector3d normal = getNormalFromPoints(a, b, c);
				double d = -normal.dot(a);

				double distance = abs(d / normal.norm());

				if (distance < closest.getDistance()) {
					closest.setDistance(distance);
					closest.setNormal(normal.normalized());
					closest.setIndices(_triangles[i]);
					closest.setVertices({ a, b, c });
				}
			}

			return closest;
		}

		void extend(Eigen::Vector3d v) {
			std::vector<Eigen::Vector3i> newTriangles;
			std::vector<Edge> edges;
			int newVertexPosition = _vertices.size();
			_vertices.push_back(v);

			for (int i = 0; i < _triangles.size(); ++i) {
				int a = _triangles[i].x();
				int b = _triangles[i].y();
				int c = _triangles[i].z();
				Eigen::Vector3d normal = getNormalFromPoints(_vertices[a], _vertices[b], _vertices[c]);

				// Point is visible by face
				if (isSameDirection(normal, v - _vertices[a])) {
					addEdge(edges, a, b);
					addEdge(edges, b, c);
					addEdge(edges, c, a);
				} else {
					newTriangles.push_back(_triangles[i]);
				}
			}

			for (int i = 0; i < edges.size(); ++i) {
				newTriangles.push_back(Eigen::Vector3i(newVertexPosition, edges[i].getA(), edges[i].getB()));
			}

			_triangles = newTriangles;
		}

		bool isCorrectOrder(int a, int b, int c, int opposite) {
			return isCorrectOrder(a, b, c, _vertices[opposite]);
		}

		bool isCorrectOrder(int a, int b, int c, Eigen::Vector3d opposite) {
			Eigen::Vector3d normal = getNormalFromPoints(_vertices[a], _vertices[b], _vertices[c]);
			Eigen::Vector3d height = opposite - _vertices[a];

			return isSameDirection(normal, height);
		}

		static void addEdge(std::vector<Edge> &edges, int a, int b) {
			for (int i = 0; i < edges.size(); ++i) {
				if (edges[i].getA() == b && edges[i].getB() == a) {
					edges.erase(edges.begin() + i);
					return;
				}
			}

			edges.push_back(Edge(a, b));
		}

	private:
		std::vector<Eigen::Vector3d> _vertices;
		std::vector<Eigen::Vector3i> _triangles;
	};
}
