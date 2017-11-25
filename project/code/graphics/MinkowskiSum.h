#pragma once

#include "CGAL.h"

namespace pbs17
{
	class MinkowskiSum {
	public:
		static bool doIntersect(Nef_Polyhedron_3 &p1, Nef_Polyhedron_3 &p2);

	private:
		static bool useMinkowskiSum;
	};
}