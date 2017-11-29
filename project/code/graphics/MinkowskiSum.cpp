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

//bool MinkowskiSum::testDistance(Polyhedron_3 &p1, Polyhedron_3 &p2) {
//	Point_3* points1 = new Point_3[p1.size_of_vertices()];
//	Point_3* points2 = new Point_3[p2.size_of_vertices()];
//
//	unsigned int i = 0;
//	for (Vertex_iterator v = p1.vertices_begin(); v != p1.vertices_end(); ++v, ++i) {
//		points1[i] = v->point();
//	}
//
//	i = 0;
//	for (Vertex_iterator v = p2.vertices_begin(); v != p2.vertices_end(); ++v, ++i) {
//		points2[i] = v->point();
//	}
//
//
//	Polytope_distance pd(points1, points1 + p1.size_of_vertices(), points2, points2 + p2.size_of_vertices());
//	assert(pd.is_valid());
//	// get squared distance (2,2,2)-(1,1,1))^2 = 3
//	std::cout << "Squared distance: " <<
//		CGAL::to_double(pd.squared_distance_numerator()) /
//		CGAL::to_double(pd.squared_distance_denominator()) << std::endl;
//
//	return true;
//}