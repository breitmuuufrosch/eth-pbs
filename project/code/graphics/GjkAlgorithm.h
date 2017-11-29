#pragma once

#include <Eigen/Core>
#include <osg/Geometry>
#include <limits>

#include "CGAL.h"
#include "Simplex.h"

namespace pbs17 {

	/**
	 * \brief Class which represents the convex-hull and manages the structures between the different frameworks.
	 */
	class GjkAlgorithm {
	public:
		static bool intersect(std::vector<Eigen::Vector3d> &convex1, std::vector<Eigen::Vector3d> &convex2) {
			//Get an initial point on the Minkowski difference.
			Eigen::Vector3d s = support(convex1, convex2, Eigen::Vector3d(1.0, 1.0, 1.0));
			std::vector<Eigen::Vector3d> sVertices;
			sVertices.push_back(s);

			//Create our initial simplex.
			Simplex simplex(sVertices);

			//Choose an initial direction toward the origin.
			Eigen::Vector3d d = -s;

			//Choose a maximim number of iterations to avoid an 
			//infinite loop during a non-convergent search.
			int maxIterations = 50;

			for (int i = 0; i < maxIterations; i++) {
				//Get our next simplex point toward the origin.
				Eigen::Vector3d a = support(convex1, convex2, d);

				//If we move toward the origin and didn't pass it 
				//then we never will and there's no intersection.
				if (isOppositeDirection(a, d)) {
					return false;
				}
				//otherwise we add the new
				//point to the simplex and
				//process it.
				simplex.add(a);
				//Here we either find a collision or we find the closest feature of
				//the simplex to the origin, make that the new simplex and update the direction
				//to move toward the origin from that feature.
				if (processSimplex(simplex, d)) {
					return true;
				}
			}
			//If we still couldn't find a simplex 
			//that contains the origin then we
			//"probably" have an intersection.
			return true;
		}

		static Eigen::Vector3d support(std::vector<Eigen::Vector3d> &convex1, std::vector<Eigen::Vector3d> &convex2, Eigen::Vector3d direction) {
			return getFurthestPoint(convex1, direction) - getFurthestPoint(convex2, -direction);
		}

		static Eigen::Vector3d getFurthestPoint(std::vector<Eigen::Vector3d> &convex1, Eigen::Vector3d direction) {
			double max = DBL_MIN;
			Eigen::Vector3d maxV(0,0,0);

			for (int i = 0; i < convex1.size(); ++i) {
				double dot = (convex1[i].dot(direction));

				if (dot > max)
				{
					max = dot;
					maxV = convex1[i];
				}
			}

			return maxV;
		}


		static bool processSimplex(Simplex &simplex, Eigen::Vector3d &direction) {
			if (simplex.count() == 2) {
				return processLine(simplex, direction);
			} else if (simplex.count() == 3) {
				return processTriangle(simplex, direction);
			} else {
				return processTetrahedron(simplex, direction);
			}
		}
	private:

		static bool isSameDirection(Eigen::Vector3d v1, Eigen::Vector3d v2)
		{
			return v1.dot(v2) > 0;
		}
		static bool isOppositeDirection(Eigen::Vector3d v1, Eigen::Vector3d v2) {
			return v1.dot(v2) < 0;
		}

		static bool processLine(Simplex &simplex, Eigen::Vector3d &direction) {
			Eigen::Vector3d a = simplex[1];
			Eigen::Vector3d b = simplex[0];
			Eigen::Vector3d ab = b - a;
			Eigen::Vector3d aO = -a;

			if (isSameDirection(ab, aO))
			{
				direction = (ab.cross(aO)).cross(ab);
			} else
			{
				simplex.remove(b);
				direction = aO;
			}

			return false;
		}

		static bool processTriangle(Simplex &simplex, Eigen::Vector3d &direction) {
			Eigen::Vector3d a = simplex[2];
			Eigen::Vector3d b = simplex[1];
			Eigen::Vector3d c = simplex[0];
			Eigen::Vector3d ab = b - a;
			Eigen::Vector3d ac = c - a;
			Eigen::Vector3d abc = ab.cross(ac);
			Eigen::Vector3d aO = -a;
			Eigen::Vector3d acNormal = abc.cross(ac);
			Eigen::Vector3d abNormal = ab.cross(abc);

			if (isSameDirection(acNormal, aO)) {
				if (isSameDirection(ac, aO)) {
					simplex.remove(b);
					direction = (ac.cross(aO)).cross(ac);
				} else {
					if (isSameDirection(ab, aO)) {
						simplex.remove(c);
						direction = (ab.cross(aO)).cross(ab);
					} else {
						simplex.remove(b);
						simplex.remove(c);
						direction = aO;
					}
				}
			} else {
				if (isSameDirection(abNormal, aO)) {
					if (isSameDirection(ab, aO)) {
						simplex.remove(c);
						direction = (ab.cross(aO)).cross(ab);
					} else {
						simplex.remove(b);
						simplex.remove(c);
						direction = aO;
					}
				} else {
					if (isSameDirection(abc, aO)) {
						direction = (abc.cross(aO)).cross(abc);
					} else {
						direction = (-abc.cross(aO)).cross(-abc);
					}
				}
			}
			return false;
		}

		static bool processTetrahedron(Simplex &simplex, Eigen::Vector3d &direction) {
			Eigen::Vector3d a = simplex[3];
			Eigen::Vector3d b = simplex[2];
			Eigen::Vector3d c = simplex[1];
			Eigen::Vector3d d = simplex[0];
			Eigen::Vector3d ac = c - a;
			Eigen::Vector3d ad = d - a;
			Eigen::Vector3d ab = b - a;
			Eigen::Vector3d bc = c - b;
			Eigen::Vector3d bd = d - b;

			Eigen::Vector3d acd = ad.cross(ac);
			Eigen::Vector3d abd = ab.cross(ad);
			Eigen::Vector3d abc = ac.cross(ab);

			Eigen::Vector3d aO = -a;

			if (isSameDirection(abc, aO)) {
				if (isSameDirection(abc.cross(ac), aO)) {
					simplex.remove(b);
					simplex.remove(d);
					direction = (ac.cross(aO)).cross(ac);
				} else if (isSameDirection(ab.cross(abc), aO)) {
					simplex.remove(c);
					simplex.remove(d);
					direction = (ab.cross(aO)).cross(ab);
				} else {
					simplex.remove(d);
					direction = abc;
				}
			} else if (isSameDirection(acd, aO)) {
				if (isSameDirection(acd.cross(ad), aO)) {
					simplex.remove(b);
					simplex.remove(c);
					direction = (ad.cross(aO)).cross(ad);
				} else if (isSameDirection(ac.cross(acd), aO)) {
					simplex.remove(b);
					simplex.remove(d);
					direction = (ac.cross(aO)).cross(ac);
				} else {
					simplex.remove(b);
					direction = acd;
				}
			} else if (isSameDirection(abd, aO)) {
				if (isSameDirection(abd.cross(ab), aO)) {
					simplex.remove(c);
					simplex.remove(d);
					direction = (ab.cross(aO)).cross(ab);
				} else if (isSameDirection(ad.cross(abd), aO)) {
					simplex.remove(b);
					simplex.remove(c);
					direction = (ad.cross(aO)).cross(ad);
				} else {
					simplex.remove(c);
					direction = abd;
				}
			} else {
				return true;
			}

			return false;
		}

	};
}
