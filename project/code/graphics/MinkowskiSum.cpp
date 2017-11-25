#include "MinkowskiSum.h"

#include <CGAL/minkowski_sum_3.h>

using namespace pbs17;

bool MinkowskiSum::useMinkowskiSum = false;

bool MinkowskiSum::doIntersect(Nef_Polyhedron_3 &p1, Nef_Polyhedron_3 &p2) {
	if (useMinkowskiSum) {
		Nef_Polyhedron_3 minkowskiSum = CGAL::minkowski_sum_3(p1, p2);
		Polyhedron_No_Id_3 sum;
		minkowskiSum.convert_to_Polyhedron(sum);

		// Construct AABB tree with a KdTree
		Tree tree(CGAL::faces(sum).first, CGAL::faces(sum).second, sum);
		tree.accelerate_distance_queries();
		// Initialize the point-in-polyhedron tester
		Point_inside inside_tester(tree);

		// Determine the side and return true if inside!
		return inside_tester(Point_3(0, 0, 0)) == CGAL::ON_BOUNDED_SIDE;
	} else {
		Nef_Polyhedron_3 intersect = p1.intersection(p2);

		return !intersect.is_empty();
	}
}